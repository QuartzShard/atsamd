//! # DMA-backed buffered UART receiver
//!
//! [`BufferedUart`] wraps a [`Uart`] and a [`RingBuffer`] to provide
//! continuous DMA-backed reception. The DMA runs a circular transfer from
//! the SERCOM DATA register into a user-provided buffer; reads copy data
//! out of the ring buffer without stopping the transfer.
//!
//! # Construction
//!
//! * [`BufferedUart::new`] — accepts any [`Receive`]-capable [`Uart`].
//! * [`BufferedUart::from_duplex`] — splits a [`Duplex`] [`Uart`] and returns
//!   the [`TxDuplex`] half alongside the [`BufferedUart`].
//!
//! The DMA channel must have [`Blocked`] interrupts (see the
//! [`ringbuffer`](crate::dmac::ringbuffer) module docs for why). Prefer the
//! lowest-numbered available channel, as Linked Descriptors can interfere with
//! other DMA trasnfers, if those transfers are not also linked and are on a
//! lower channel. (errata DMA101-6).
//!
//! # Blocking reads
//!
//! [`embedded_io::Read`] spins until data is available, then copies from
//! the ring buffer. [`embedded_io::ReadReady`] returns `true` when
//! [`available`](BufferedUart::available) `> 0`. SERCOM status errors
//! (parity, framing, buffer overflow) are checked on each read.
//!
//! # Async reads
//!
//! <span class="stab portability" title="Available on crate feature `async`
//! only"><code>async</code></span>
//!
//! Call [`into_future`](BufferedUart::into_future) with a [`Binding`] proof
//! to obtain [`embedded_io_async::Read`]. The SERCOM RXC interrupt is used
//! as the wakeup source. The original blocking [`embedded_io::Read`] and
//! [`embedded_io::ReadReady`] impls remain usable.
//!
//! # Overflow
//!
//! Overflow is detected by the underlying [`RingBuffer`]'s [`WrapOffset`]
//! state machine. Read operations return
//! `Err(`[`Error::Overflow`]`)` if the DMA has lapped the read pointer.
//! Call [`clear`](BufferedUart::clear) to discard corrupted data and resume.
//!
//! # Example
//!
//! ```ignore
//! let (buf_uart, tx) = BufferedUart::from_duplex(uart, &mut dma_buf, rx_channel);
//!
//! let mut tmp = [0u8; 64];
//! loop {
//!     match buf_uart.read(&mut tmp) {
//!         Ok(n) => process(&tmp[..n]),
//!         Err(Error::Overflow) => buf_uart.clear(),
//!         Err(e) => handle_error(e),
//!     }
//! }
//! ```
//!
//! [`Binding`]: crate::async_hal::interrupts::Binding
//! [`WrapOffset`]: crate::dmac::ringbuffer::WrapOffset

use atsamd_hal_macros::hal_macro_helper;

use crate::{
    dmac::{
        AnyChannel, Blocked, Channel, Ready, TriggerAction,
        ringbuffer::{RingBuffer, RingBufferError},
    },
    sercom::{
        Sercom,
        dma::SercomPtr,
        uart::{Duplex, Error, Receive, RxDuplex, TxDuplex, Uart, ValidConfig},
    },
    typelevel::NoneT,
};
use core::marker::PhantomData;

/// DMA-backed buffered UART receiver.
///
/// Wraps a [`Uart`] and a [`RingBuffer`] for continuous circular DMA
/// reception. The type parameter `I` is [`NoneT`] for blocking mode, or a
/// [`Binding`](crate::async_hal::interrupts::Binding) proof for async mode
/// (see [`into_future`](Self::into_future)).
pub struct BufferedUart<'buf, C, D, Ch, I = NoneT>
where
    C: ValidConfig<Word = u8>,
    D: Receive,
    Ch: AnyChannel<Interrupts = Blocked>,
{
    uart: Uart<C, D, NoneT, NoneT>,
    ringbuf: RingBuffer<'buf, u8, Ch, SercomPtr<u8>>,
    _isr: PhantomData<I>,
}

// Generic constructor — works with any D: Receive
impl<'buf, C, D, Ch, S, I> BufferedUart<'buf, C, D, Ch, I>
where
    C: ValidConfig<Sercom = S, Word = u8>,
    D: Receive,
    Ch: AnyChannel<Interrupts = Blocked>,
    S: Sercom,
{
    /// Construct a [`BufferedUart`] from a [`Uart`], DMA channel, and buffer.
    ///
    /// The [`Uart`] must not have DMA channels attached. The provided DMA
    /// channel will be used for continuous circular reception into `buf`.
    #[hal_macro_helper]
    pub fn new(
        uart: Uart<C, D, NoneT, NoneT>,
        buf: &'buf mut [u8],
        channel: Channel<Ch::Id, Ready, Ch::Interrupts>,
    ) -> Self {
        let sercom_ptr = uart.sercom_ptr();

        #[hal_cfg("sercom0-d5x")]
        let trigger_action = TriggerAction::Burst;

        #[hal_cfg(any("sercom0-d11", "sercom0-d21"))]
        let trigger_action = TriggerAction::Beat;

        let ringbuf = RingBuffer::new(buf, sercom_ptr, channel, S::DMA_RX_TRIGGER, trigger_action);
        Self {
            uart,
            ringbuf,
            _isr: PhantomData,
        }
    }

    /// Stop the DMA transfer and return the [`Uart`], DMA channel, and
    /// buffer.
    #[allow(clippy::type_complexity)]
    pub fn free(
        self,
    ) -> (
        Uart<C, D, NoneT, NoneT>,
        Channel<Ch::Id, Ready, Ch::Interrupts>,
        &'buf mut [u8],
    ) {
        let (chan, _sercom_ptr, buf) = self.ringbuf.free();
        (self.uart, chan, buf)
    }

    /// Discard all buffered data and reset overflow state.
    pub fn clear(&mut self) {
        self.ringbuf.clear()
    }

    /// Return the number of bytes available to read, or `Err` on overflow.
    pub fn available(&mut self) -> Result<usize, RingBufferError> {
        self.ringbuf.available()
    }
}

// Specialized constructor — splits Duplex for the user
impl<'buf, C, Ch, S> BufferedUart<'buf, C, RxDuplex, Ch>
where
    C: ValidConfig<Sercom = S, Word = u8>,
    Ch: AnyChannel<Interrupts = Blocked>,
    S: Sercom,
{
    /// Construct a [`BufferedUart`] from a full-[`Duplex`] [`Uart`],
    /// splitting it and returning the [`TxDuplex`] half for the caller.
    #[allow(clippy::type_complexity)]
    pub fn from_duplex(
        uart: Uart<C, Duplex, NoneT, NoneT>,
        buf: &'buf mut [u8],
        channel: Channel<Ch::Id, Ready, Ch::Interrupts>,
    ) -> (Self, Uart<C, TxDuplex, NoneT, NoneT>) {
        let (rx, tx) = uart.split();
        let buffered = Self::new(rx, buf, channel);
        (buffered, tx)
    }
}

// embedded_io trait impls
impl<'buf, C, D, Ch, I> embedded_io::ErrorType for BufferedUart<'buf, C, D, Ch, I>
where
    C: ValidConfig<Word = u8>,
    D: Receive,
    Ch: AnyChannel<Interrupts = Blocked>,
{
    type Error = Error;
}

impl<'buf, C, D, Ch, S, I> embedded_io::Read for BufferedUart<'buf, C, D, Ch, I>
where
    C: ValidConfig<Sercom = S, Word = u8>,
    D: Receive,
    Ch: AnyChannel<Interrupts = Blocked>,
    S: Sercom,
{
    /// Block until at least one byte is available, then read into `buf`.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        use embedded_io::ReadReady;
        self.uart.read_flags_errors()?;
        while !self.read_ready()? {
            core::hint::spin_loop();
        }
        self.ringbuf.read(buf).map_err(Self::Error::from)
    }
}

impl<'buf, C, D, Ch, S, I> embedded_io::ReadReady for BufferedUart<'buf, C, D, Ch, I>
where
    C: ValidConfig<Sercom = S, Word = u8>,
    D: Receive,
    Ch: AnyChannel<Interrupts = Blocked>,
    S: Sercom,
{
    /// Returns `true` if there is data available to read.
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ringbuf.available()? >= 1)
    }
}

impl From<RingBufferError> for Error {
    fn from(val: RingBufferError) -> Error {
        match val {
            RingBufferError::Overflow => Error::Overflow,
        }
    }
}

#[cfg(feature = "async")]
mod async_impl {
    use super::*;
    use crate::{
        async_hal::interrupts::{Binding, InterruptSource},
        sercom::uart::{Flags, InterruptHandler},
    };
    use core::{future::poll_fn, task::Poll};
    use embedded_io_async::Read;

    impl<'buf, C, D, Ch, S> BufferedUart<'buf, C, D, Ch>
    where
        C: ValidConfig<Sercom = S, Word = u8>,
        D: Receive,
        Ch: AnyChannel<Interrupts = Blocked>,
        S: Sercom,
    {
        /// Enable async reads by binding the SERCOM interrupt.
        ///
        /// Returns a [`BufferedUart`] with [`embedded_io_async::Read`]
        /// available. The original blocking [`embedded_io::Read`] and
        /// [`embedded_io::ReadReady`] impls remain usable.
        pub fn into_future<I>(self, _irqs: I) -> BufferedUart<'buf, C, D, Ch, I>
        where
            I: Binding<S::Interrupt, InterruptHandler<S>>,
        {
            S::Interrupt::unpend();
            unsafe { S::Interrupt::enable() };
            BufferedUart {
                _isr: PhantomData,
                uart: self.uart,
                ringbuf: self.ringbuf,
            }
        }
    }

    impl<'buf, C, D, Ch, S, I> Read for BufferedUart<'buf, C, D, Ch, I>
    where
        C: ValidConfig<Sercom = S, Word = u8>,
        D: Receive,
        Ch: AnyChannel<Interrupts = Blocked>,
        S: Sercom,
        I: Binding<S::Interrupt, InterruptHandler<S>>,
    {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            poll_fn(|cx| match self.ringbuf.available() {
                Ok(0) => {
                    self.uart.disable_interrupts(Flags::RXC);
                    S::rx_waker().register(cx.waker());
                    self.uart.enable_interrupts(Flags::RXC);
                    match self.ringbuf.available() {
                        Ok(0) => Poll::Pending,
                        Ok(_) => Poll::Ready(Ok(())),
                        Err(e) => Poll::Ready(Err(e)),
                    }
                }
                Ok(_) => Poll::Ready(Ok(())),
                Err(e) => Poll::Ready(Err(e)),
            })
            .await?;
            self.ringbuf.read(buf).map_err(Self::Error::from)
        }
    }
}
