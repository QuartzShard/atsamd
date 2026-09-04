//! # Device Service Unit
//!
//! This module allows users to interact with a DSU peripheral.
//!
//! - Run a CRC32 checksum over memory
//! - Erase the entire chip ([`Dsu::chip_erase`])
//! - Query the device protection state
#![warn(missing_docs)]

use core::convert::Infallible;

use crate::pac::{self, Pac};

/// Device Service Unit
pub struct Dsu {
    /// PAC peripheral
    dsu: pac::Dsu,
}

/// Errors from hardware
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PeripheralError {
    /// Usually misaligned address of length
    BusError,
}

/// Error from within the DSU
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Address or length was not word aligned
    AlignmentError,
    /// The PAC would not unlock the DSU for us
    PacUnlockFailed,
    /// CRC32 operation failed
    CrcFailed,
    /// The chip-erase command is locked (NVMCTRL `CELCK` command issued)
    ChipEraseLocked,
    /// Hardware-generated errors
    Peripheral(PeripheralError),
}

/// NVM result type
pub type Result<T> = core::result::Result<T, Error>;

impl Dsu {
    /// Unlock the DSU and instantiate peripheral
    #[inline]
    pub fn new(dsu: pac::Dsu, pac: &Pac) -> Result<Self> {
        // Attempt to unlock DSU
        pac.wrctrl()
            .write(|w| unsafe { w.perid().bits(33).key().clr() });

        // Check if DSU was unlocked
        if pac.statusb().read().dsu_().bit_is_set() {
            Err(Error::PacUnlockFailed)
        } else {
            Ok(Self { dsu })
        }
    }

    /// Clear bus error bit
    fn clear_bus_error(&mut self) {
        self.dsu.statusa().write(|w| w.berr().set_bit());
    }

    /// Read bus error bit
    fn bus_error(&mut self) -> bool {
        self.dsu.statusa().read().berr().bit()
    }

    /// Check if operation is done
    fn is_done(&self) -> bool {
        self.dsu.statusa().read().done().bit_is_set()
    }

    /// Check if an operation has failed
    fn has_failed(&self) -> bool {
        self.dsu.statusa().read().fail().bit_is_set()
    }

    /// Set target address given as number of words offset
    fn set_address(&mut self, address: u32) -> Result<()> {
        self.dsu.addr().write(|w| unsafe { w.addr().bits(address) });
        Ok(())
    }

    /// Set length given as number of words
    fn set_length(&mut self, length: u32) -> Result<()> {
        self.dsu
            .length()
            .write(|w| unsafe { w.length().bits(length) });
        Ok(())
    }

    /// Seed CRC32
    fn seed(&mut self, data: u32) {
        self.dsu.data().write(|w| unsafe { w.data().bits(data) });
    }

    /// Calculate CRC32 of a memory region
    ///
    /// - `address` is an address within a flash; must be word-aligned
    /// - `length` is a length of memory region that is being processed. Must be
    ///   word-aligned
    #[inline]
    pub fn crc32(&mut self, address: u32, length: u32) -> Result<u32> {
        // The algorithm employed is the industry standard CRC32 algorithm using the
        // generator polynomial 0xEDB88320
        // (reversed representation of 0x04C11DB7).
        //
        // https://crccalc.com/, Hex input same as memory contents, Calc CRC-32
        // but output is reversed

        if address % 4 != 0 {
            return Err(Error::AlignmentError);
        }

        if length % 4 != 0 {
            return Err(Error::AlignmentError);
        }

        let num_words = length / 4;

        // Calculate target flash address
        let flash_address = address / 4;

        // Set the ADDR of where to start calculation, as number of words
        self.set_address(flash_address)?;

        // Amount of words to check
        self.set_length(num_words)?;

        // Set CRC32 seed
        self.seed(0xffff_ffff);

        // Clear the status flags indicating termination of the operation
        self.dsu
            .statusa()
            .write(|w| w.done().set_bit().fail().set_bit());

        // Initialize CRC calculation
        self.dsu.ctrl().write(|w| w.crc().set_bit());

        // Wait until done or failed
        while !self.is_done() && !self.has_failed() {}
        if self.has_failed() {
            return Err(Error::CrcFailed);
        }

        // CRC startup generated a bus error
        // Generally misaligned length or address
        if self.bus_error() {
            // Return the reported bus error and clear it
            self.clear_bus_error();
            Err(Error::Peripheral(PeripheralError::BusError))
        } else {
            // Return the calculated CRC32 (complement of data register)
            Ok(!self.dsu.data().read().data().bits())
        }
    }

    /// Check whether the device is protected by the NVMCTRL security bit
    ///
    /// While protected, external debugger access to memories and most DSU
    /// commands is restricted. The security bit is set with
    /// [`Nvm::enable_security_bit`](crate::nvm::Nvm::enable_security_bit)
    /// and only cleared by a chip erase.
    #[inline]
    pub fn is_protected(&self) -> bool {
        self.dsu.statusb().read().prot().bit_is_set()
    }

    /// Check whether the chip-erase command is locked
    ///
    /// While locked, [`Dsu::chip_erase`] and the equivalent debugger-issued
    /// command are unavailable. The lock is controlled from firmware with
    /// [`Nvm::enable_chip_erase_lock`](crate::nvm::Nvm::enable_chip_erase_lock)
    /// and
    /// [`Nvm::disable_chip_erase_lock`](crate::nvm::Nvm::disable_chip_erase_lock).
    #[inline]
    pub fn chip_erase_locked(&self) -> bool {
        self.dsu.statusb().read().celck().bit_is_set()
    }

    /// Erase the entire chip
    ///
    /// Clears all volatile memories (RAM) and erases the whole flash array
    /// (including the SmartEEPROM emulation area) simultaneously, then clears
    /// the NVMCTRL security bit and the chip-erase lock, leaving the device
    /// unprotected. The `BOOTPROT` bootloader section and the auxiliary pages
    /// (calibration, factory and user pages) are not affected.
    ///
    /// On success this function never returns: the erase destroys the running
    /// program and its stack while the DSU completes the operation
    /// independently of the CPU. The device does not run any further code and
    /// must be recovered with an external reset or power cycle. If a
    /// bootloader is protected by `BOOTPROT`, it will run again afterwards.
    ///
    /// # Errors
    ///
    /// Returns [`Error::ChipEraseLocked`] (without side effects) if the
    /// chip-erase command is locked; unlock it first with
    /// [`Nvm::disable_chip_erase_lock`](crate::nvm::Nvm::disable_chip_erase_lock).
    ///
    /// # Safety
    ///
    /// This is a point of no return that destroys all data and firmware
    /// outside the `BOOTPROT` section. Additionally, the caller must ensure
    /// that:
    ///
    /// - Interrupts are disabled before calling, so no handler runs from erased
    ///   memory while the operation is in flight.
    /// - No reset can occur mid-erase — in particular the watchdog must be
    ///   disabled (and not fused always-on). A reset during the erase aborts it
    ///   and leaves partially-erased blocks in an unknown state.
    ///
    /// ```ignore
    /// nvm.disable_chip_erase_lock()?; // if the application had locked it
    /// cortex_m::interrupt::disable();
    /// unsafe { dsu.chip_erase()? }; // Ok is never returned
    /// ```
    pub unsafe fn chip_erase(&mut self) -> Result<Infallible> {
        // The hardware silently discards CTRL.CE while locked; check first so
        // the caller gets an error instead of a hang
        if self.chip_erase_locked() {
            return Err(Error::ChipEraseLocked);
        }

        // Clear the status flags indicating termination of the operation
        self.dsu.statusa().write(|w| {
            w.done().set_bit();
            w.fail().set_bit();
            w.berr().set_bit();
            w.perr().set_bit()
        });

        // Start the chip erase; the DSU is an independent AHB master and
        // completes it regardless of what happens to the CPU
        self.dsu.ctrl().write(|w| w.ce().set_bit());

        // RAM (including this stack) is being cleared and flash erased under
        // us; spin without touching memory until the CPU inevitably dies
        loop {
            core::hint::spin_loop();
        }
    }
}
