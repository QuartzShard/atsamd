//! # Circular DMA ring buffer
//!
//! [`RingBuffer`] wraps a circular [`Transfer`] to continuously receive data
//! from a non-incrementing peripheral source (such as a SERCOM DATA register).
//! The DMA hardware writes incoming data into the buffer in a loop, and the
//! application reads it out at its own pace via [`peek`](RingBuffer::peek),
//! [`read`](RingBuffer::read), or [`consume`](RingBuffer::consume).
//!
//! The DMA write position is determined by reading the `BTCNT` field from the
//! DMAC writeback descriptor in SRAM. Data is copied out of the buffer using
//! per-element [`ptr::read_volatile`] — a `&[T]` reference into the buffer
//! is never created, because the DMA hardware is mutating it concurrently.
//!
//! # Blocked interrupts
//!
//! [`RingBuffer`] requires `Interrupts = Blocked` on its DMA channel.
//! The async DMAC interrupt handler disables the channel on every TCMPL
//! interrupt. For circular transfers, TCMPL fires at the end of every buffer
//! cycle, so the handler would kill the transfer after the first wrap.
//! [`Blocked`] channels cannot have their interrupts enabled or disabled by
//! user code. Use [`available`](RingBuffer::available) to poll for new data.
//!
//! # Overflow detection
//!
//! A [`WrapOffset`] state machine tracks how many times the DMA write pointer
//! has wrapped relative to the read pointer:
//!
//! * `None` — write and read are on the same lap. Normal initial state.
//! * `Wrapped` — write has wrapped once ahead of read. This is normal operation
//!   for a circular buffer.
//! * `Overrun` — write has wrapped twice or more, or has overtaken read. Data
//!   was lost. This state is sticky; call [`clear`](RingBuffer::clear) to
//!   discard corrupted data and resume.
//!
//! [`peek`](RingBuffer::peek) and [`read`](RingBuffer::read) check for
//! overflow and return `Err(RingBufferError::Overflow)` if the buffer is in
//! the `Overrun` state.
//!
//! # Timing constraints
//!
//! Data is copied element-by-element with `read_volatile`. Each element
//! takes roughly 3 CPU cycles (`LDR` + `STR` + loop overhead), so the
//! worst-case time to drain a full buffer is:
//!
//! ```text
//! drain_time = buf_len * 3 / cpu_freq
//! ```
//!
//! The buffer fill time depends on the peripheral's data rate:
//!
//! ```text
//! fill_time  = buf_len / byte_rate
//! ```
//!
//! For UART, `byte_rate ≈ baud / 10` (8N1 framing). As long as drain time
//! is well below fill time, the ring buffer is sound for your use case.
//!
//! | CPU clock | Buffer | Drain time | Baud rate | Fill time | Headroom |
//! |-----------|--------|------------|-----------|-----------|----------|
//! | 48 MHz    | 256 B  | ~16 µs     | 9600      | ~267 ms   | ~16700×  |
//! | 48 MHz    | 256 B  | ~16 µs     | 115200    | ~22 ms    | ~1390×   |
//! | 48 MHz    | 1024 B | ~64 µs     | 115200    | ~89 ms    | ~1390×   |
//! | 48 MHz    | 2048 B | ~128 µs    | 115200    | ~178 ms   | ~1390×   |
//! | 48 MHz    | 2048 B | ~128 µs    | 1 Mbaud   | ~20 ms    | ~160×    |
//! | 48 MHz    | 2048 B | ~128 µs    | 3 Mbaud   | ~6.8 ms   | ~53×     |
//! | 120 MHz   | 2048 B | ~51 µs     | 3 Mbaud   | ~6.8 ms   | ~133×    |
//!
//! If the peripheral's data rate can fill the buffer faster than the CPU can
//! drain it, or if the DMA wraps between the overflow check and the end of a
//! [`read`](RingBuffer::read) call, data may be silently corrupted. For
//! higher-throughput peripherals, consider a double-buffer approach.
//!
//! You must also poll frequently enough that the DMA cannot complete a full
//! buffer cycle between polls, or overflow detection may miss a wrap. The
//! maximum poll interval is:
//!
//! ```text
//! max_poll_interval = buf_len / byte_rate
//! ```
//!
//! For example, a 2048 byte buffer at 115200 baud can go ~178 ms between
//! polls; at 3 Mbaud, poll at least every ~5 ms.
//!
//! # DMA errata (SAMD5x/E5x)
//!
//! Silicon errata DMA101-6: when a channel using linked descriptors is
//! already active, enabling a new channel whose descriptor is not linked (or
//! is circular / self-linked) can cause a Fetch Error (FERR) if the new
//! channel number is lower than the active one.
//!
//! Because [`RingBuffer`] uses a circular (self-linked) descriptor, allocate
//! it to the lowest-numbered DMA channel in your application. For example, if
//! you use channels 0–3, put the ring buffer on channel 0 and use 1–3 for
//! linked-descriptor transfers.
//!
//! # Example
//!
//! ```ignore
//! let mut ringbuf = RingBuffer::new(
//!     dma_buf,
//!     uart.sercom_ptr(),
//!     rx_channel,
//!     TriggerSource::Sercom0Rx,
//!     TriggerAction::Beat,
//! );
//!
//! let mut tmp = [0u8; 64];
//! loop {
//!     match ringbuf.read(&mut tmp) {
//!         Ok(n) => process(&tmp[..n]),
//!         Err(RingBufferError::Overflow) => ringbuf.clear(),
//!     }
//! }
//! ```
//! # Future work
//!
//! A specialised impl for [`Available`](crate::dmac::Available) interrupt
//! channels could use TCMPL to count wraps when the async DMAC handler is
//! not in use.

use crate::dmac::{
    AnyChannel, Beat, Blocked, Buffer, BusyTransfer, ChId, Channel, ChannelId, ChannelInterrupts,
    Ready, Transfer, TriggerAction, TriggerSource, sram,
};

use core::{mem::ManuallyDrop, ptr};

/// Circular DMA ring buffer, continuously receiving from a peripheral
/// into a [`Transfer`] and providing polling-based read access.
/// The `ring` pointer aliases the buffer owned by the inner transfer;
/// all reads go through [`ptr::read_volatile`].
pub struct RingBuffer<'buf, T, C, S>
where
    C: AnyChannel<Interrupts = Blocked>,
    S: Buffer<Beat = T>,
    T: Copy + Beat,
{
    transfer: ManuallyDrop<BusyTransfer<C, S, &'buf mut [T]>>,
    ring: ptr::NonNull<T>,
    len: usize,
    read_index: usize,
    write_index: usize,
    wrap: WrapOffset,
}

impl<'buf, T, C, S> RingBuffer<'buf, T, C, S>
where
    C: AnyChannel<Interrupts = Blocked>,
    S: Buffer<Beat = T>,
    T: Copy + Beat,
{
    /// Construct a new [`RingBuffer`] and arm the DMA transfer.
    ///
    /// # Panics
    ///
    /// * If `periph` is an incrementing source (only non-incrementing sources
    ///   like SERCOM DATA registers are valid).
    /// * If `buf` is null.
    pub fn new<
        Ch: AnyChannel<Id = ChannelId<C>, Status = Ready, Interrupts = ChannelInterrupts<C>>,
    >(
        buf: &'buf mut [T],
        periph: S,
        channel: Ch,
        trigger_src: TriggerSource,
        trigger_act: TriggerAction,
    ) -> Self {
        assert!(periph.buffer_len() == 1);

        let ring = ptr::NonNull::new(buf.as_mut_ptr()).expect("User provided a null buffer");
        let len = buf.len();

        let transfer = ManuallyDrop::new(unsafe {
            Transfer::new_unchecked(channel, periph, buf, true).begin(trigger_src, trigger_act)
        });

        Self {
            transfer,
            ring,
            len,
            read_index: 0,
            write_index: 0,
            wrap: WrapOffset::None,
        }
    }

    /// Read BTCNT from dmac SRAM to get the current location of the "write ptr"
    fn sync_write_index(&mut self) -> usize {
        // SAFETY: We know where the channel is (in the transfer), we just want to read
        // BTCNT. We do not create a reference from this ptr
        let descriptor: sram::DmacDescriptor =
            unsafe { ptr::read_volatile(sram::writeback_addr().add(ChannelId::<C>::USIZE)) };
        let new = self.len - descriptor.btcnt as usize;
        if new < self.write_index {
            self.wrap.inc()
        }
        self.write_index = new;
        self.write_index
    }

    /// Get count of available `T`s in the buffer
    pub fn available(&mut self) -> Result<usize, RingBufferError> {
        self.check_overflow()?;
        Ok(self.len * self.wrap as usize + self.write_index - self.read_index)
    }

    /// Copy up to `buf.len()` `T`s out of the [`RingBuffer`], without advancing
    /// the read index. Returns the number of bytes read, which is
    /// min(buf.len(), available)
    pub fn peek(&mut self, buf: &mut [T]) -> Result<usize, RingBufferError> {
        let readlen = buf.len().min(self.available()?);
        for (i, slot) in buf.iter_mut().enumerate().take(readlen) {
            *slot = unsafe {
                let elem = self.ring.as_ptr().add((self.read_index + i) % self.len);
                ptr::read_volatile(elem)
            };
        }
        Ok(readlen)
    }
    /// Copy up to `buf.len()` `T`s out of the [`RingBuffer`], and advance the
    /// read index. Returns the number of elements read, which is
    /// `min(buf.len(), available)`.
    pub fn read(&mut self, buf: &mut [T]) -> Result<usize, RingBufferError> {
        let read = self.peek(buf)?;
        self.consume(read)?;
        Ok(read)
    }

    /// Drain `n` `T`s out of the read buffer (advances the read index without
    /// reading anything)
    pub fn consume(&mut self, n: usize) -> Result<(), RingBufferError> {
        let readlen = n.min(self.available()?);
        let new = (self.read_index + readlen) % self.len;
        if new < self.read_index {
            self.wrap.dec()
        }
        self.read_index = new;
        Ok(())
    }
    /// Clears the data in the buffer (sets read_index = write_index). Required
    /// to recover from overruns.
    pub fn clear(&mut self) {
        self.read_index = self.sync_write_index();
        self.wrap = WrapOffset::None;
    }

    /// Stop the transfer and return the components of the buffer.
    /// [`RingBuffer`]'s [`Drop`] impl will also stop the transfer for
    /// safety, but will discard the buffer and source
    #[allow(clippy::type_complexity)]
    pub fn free(self) -> (Channel<C::Id, Ready, C::Interrupts>, S, &'buf mut [T]) {
        // Suppress the Drop impl
        let mut this = ManuallyDrop::new(self);
        // SAFETY: Drop will not run due to ManuallyDrop, so this is the last time
        // anything can alias the contents of `this`
        let xfer = unsafe { ManuallyDrop::take(&mut this.transfer) };
        xfer.stop()
    }

    /// Returns `Err(RingBufferError::Overflow)` if the write index has
    /// lapped the read index.
    fn check_overflow(&mut self) -> Result<(), RingBufferError> {
        self.sync_write_index();
        match self.wrap {
            WrapOffset::Overrun => Err(RingBufferError::Overflow),
            WrapOffset::Wrapped if self.write_index > self.read_index => {
                Err(RingBufferError::Overflow)
            }
            WrapOffset::Wrapped | WrapOffset::None => Ok(()),
        }
    }
}

impl<'buf, T, C, S> Drop for RingBuffer<'buf, T, C, S>
where
    C: AnyChannel<Interrupts = Blocked>,
    S: Buffer<Beat = T>,
    T: Copy + Beat,
{
    fn drop(&mut self) {
        // SAFETY: We're dropping, nothing to contend this alias
        let xfer = unsafe { ManuallyDrop::take(&mut self.transfer) };
        xfer.stop();
    }
}

/// SAFETY: RingBuffer does not alias or provide sharing. It holds &'buf mut to
/// its' data.
unsafe impl<'buf, T, C, S> Send for RingBuffer<'buf, T, C, S>
where
    T: Copy + Beat + Send,
    C: AnyChannel<Interrupts = Blocked> + Send,
    S: Buffer<Beat = T> + Send,
{
}

#[repr(u8)]
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
/// Tracks how many times the DMA write pointer has wrapped relative to
/// the read pointer. Incremented when write wraps, decremented when read
/// wraps. `Overrun` is sticky and saturates — call [`RingBuffer::clear`]
/// to recover.
pub enum WrapOffset {
    None = 0,
    Wrapped = 1,
    Overrun = 2,
}

impl WrapOffset {
    fn inc(&mut self) {
        *self = match self {
            Self::None => Self::Wrapped,
            Self::Wrapped => Self::Overrun,
            Self::Overrun => Self::Overrun,
        }
    }
    fn dec(&mut self) {
        *self = match self {
            Self::None => panic!("Invalid to wrap read before write"),
            Self::Wrapped => Self::None,
            // Sticky, as captures 2+ potential overruns.
            Self::Overrun => Self::Overrun,
        }
    }
}

/// Errors returned by [`RingBuffer`] read operations.
pub enum RingBufferError {
    /// The DMA write pointer has lapped the read pointer. Data was lost.
    /// Call [`RingBuffer::clear`] to discard corrupted data and resume.
    Overflow,
}
