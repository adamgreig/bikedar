#![feature(asm)]
#![feature(used)]
#![no_std]

#[macro_use]
extern crate cortex_m;
extern crate cortex_m_rt;
extern crate stm32f7;
extern crate smoltcp;

use core::u16;
use cortex_m::asm;
use stm32f7::stm32f7x7::{GPIOA, GPIOB, GPIOC, GPIOD, GPIOG, USART3, TIM5, DMA1};
use stm32f7::stm32f7x7::{RCC, PWR, SYSCFG, FLASH, ETHERNET_MAC, ETHERNET_DMA};
use smoltcp::phy::Device;
use smoltcp::iface::{ArpCache, SliceArpCache, EthernetInterface};
use smoltcp::socket::{AsSocket, SocketSet};
use smoltcp::socket::{TcpSocket, TcpSocketBuffer};
use smoltcp::wire::{EthernetAddress, IpAddress};

static mut BENCH_BUF: [u32; 8] = [0x00FF_FFFF; 8];

#[repr(C,packed)]
#[derive(Copy,Clone)]
struct TDes {
    tdes0: u32,
    tdes1: u32,
    tdes2: u32,
    tdes3: u32,
}

#[repr(C,packed)]
#[derive(Copy,Clone)]
struct RDes {
    rdes0: u32,
    rdes1: u32,
    rdes2: u32,
    rdes3: u32,
}

// Assign buffers as u32 to ensure alignment, but we'll use them as u8
const ETH_BUF_LEN: usize = 2;
static mut TBUF: [[u32; 2048/4]; ETH_BUF_LEN] = [[0; 2048/4]; ETH_BUF_LEN];
static mut RBUF: [[u32; 2048/4]; ETH_BUF_LEN] = [[0; 2048/4]; ETH_BUF_LEN];
static mut TD: [TDes; ETH_BUF_LEN] = [TDes {tdes0: 0, tdes1: 0, tdes2: 0, tdes3: 0}; ETH_BUF_LEN];
static mut RD: [RDes; ETH_BUF_LEN] = [RDes {rdes0: 0, rdes1: 0, rdes2: 0, rdes3: 0}; ETH_BUF_LEN];
static mut TDPTR: usize = 0;
static mut RDPTR: usize = 0;

fn transmit_packet(buf: &[u8]) {
    unsafe {
        for td in TD.iter_mut() {
            if buf.as_ptr() as u32 == td.tdes2 as u32 {
                usart_send_string("TX\r\n");
                td.tdes1 = buf.len() as u32;
                td.tdes0 |= 1<<31;
                break;
            }
        }

        // Check if the TX DMA has suspended due to no new packets
        cortex_m::interrupt::free(|cs| {
            let eth_dma = ETHERNET_DMA.borrow(cs);
            if eth_dma.dmasr.read().tps().bits() == 0b110 {
                // Wait for our update to TDES0 then request a restart
                asm::dsb();
                eth_dma.dmatpdr.write(|w| w.tpd().bits(0));
            }
        });
    }
}

fn release_packet(buf: &[u8]) {
    // Find the RDes that held this buffer and release it to the DMA
    unsafe {
        for rd in RD.iter_mut() {
            if buf.as_ptr() as u32 == rd.rdes2 {
                rd.rdes0 |= 1<<31;
                break;
            }
        }

        // Check if the RX DMA has suspended due to all buffers taken
        cortex_m::interrupt::free(|cs| {
            let eth_dma = ETHERNET_DMA.borrow(cs);
            if eth_dma.dmasr.read().rps().bits() == 0b100 {
                // Wait for our update to RDES0 then request a restart
                asm::dsb();
                eth_dma.dmarpdr.write(|w| w.rpd().bits(0));
            }
        });
    }
}

struct EthernetRxBuffer(&'static [u8]);
struct EthernetTxBuffer(&'static mut [u8]);

impl AsRef<[u8]> for EthernetTxBuffer { fn as_ref(&self) -> &[u8] { self.0 } }
impl AsRef<[u8]> for EthernetRxBuffer { fn as_ref(&self) -> &[u8] { self.0 } }
impl AsMut<[u8]> for EthernetTxBuffer { fn as_mut(&mut self) -> &mut [u8] { self.0 } }

impl Drop for EthernetTxBuffer {
    fn drop(&mut self) {
        transmit_packet(self.0);
    }
}

impl Drop for EthernetRxBuffer {
    fn drop(&mut self) {
        release_packet(self.0);
    }
}

struct EthernetDevice {}

impl Device for EthernetDevice {
    type RxBuffer = EthernetRxBuffer;
    type TxBuffer = EthernetTxBuffer;

    fn mtu(&self) -> usize { 1536 }

    fn receive(&mut self) -> Result<Self::RxBuffer, smoltcp::Error> {
        asm::dmb();

        // See if the next RDes has been released by the DMA
        unsafe {
            if RD[RDPTR].rdes0 & (1<<31) == 0 {
                usart_send_string("RX\r\n");
                let len = (RD[RDPTR].rdes0 >> 16) & 0x3FFF;
                let buf = core::slice::from_raw_parts(
                    RD[RDPTR].rdes2 as *const u8, len as usize);
                RDPTR = (RDPTR + 1) % ETH_BUF_LEN;
                return Ok(EthernetRxBuffer(buf));
            }

            Err(smoltcp::Error::Exhausted)
        }
    }

    fn transmit(&mut self, length: usize) -> Result<Self::TxBuffer, smoltcp::Error> {
        asm::dmb();

        // Look for an TDes that have been released by the DMA
        unsafe {
            if TD[TDPTR].tdes0 & (1<<31) == 0 {
                let buf = core::slice::from_raw_parts_mut(
                    TD[TDPTR].tdes2 as *mut u8, length);
                TDPTR = (TDPTR + 1) % ETH_BUF_LEN;
                return Ok(EthernetTxBuffer(buf));
            }

            Err(smoltcp::Error::Exhausted)
        }
    }
}

fn gpio_init() {
    cortex_m::interrupt::free(|cs| {
        let gpioa = GPIOA.borrow(cs);
        let gpiob = GPIOB.borrow(cs);
        let gpioc = GPIOC.borrow(cs);
        let gpiod = GPIOD.borrow(cs);
        let gpiog = GPIOG.borrow(cs);

        // GPIOA 1, 2, 7
        // GPIOB 13
        // GPIOC 1, 4, 5
        // GPIOG 11, 13
        //
        // All set to alternate function 11 and very high speed.

        gpioa.moder.modify(|_, w|
            w.moder1().alternate()
             .moder2().alternate()
             .moder7().alternate());

        gpiob.moder.modify(|_, w|
             w.moder13().alternate());

        gpioc.moder.modify(|_, w|
            w.moder1().alternate()
             .moder4().alternate()
             .moder5().alternate());

        gpiod.moder.modify(|_, w|
            w.moder8().alternate()
             .moder9().alternate());

        gpiog.moder.modify(|_, w|
            w.moder11().alternate()
             .moder13().alternate());

        gpioa.ospeedr.modify(|_, w|
            w.ospeedr1().very_high_speed()
             .ospeedr2().very_high_speed()
             .ospeedr7().very_high_speed());

        gpiob.ospeedr.modify(|_, w|
            w.ospeedr13().very_high_speed());

        gpioc.ospeedr.modify(|_, w|
            w.ospeedr1().very_high_speed()
             .ospeedr4().very_high_speed()
             .ospeedr5().very_high_speed());

        gpiog.ospeedr.modify(|_, w|
            w.ospeedr11().very_high_speed()
             .ospeedr13().very_high_speed());

        gpioa.afrl.modify(|_, w|
            w.afrl1().af11()
             .afrl2().af11()
             .afrl7().af11());

        gpiob.afrh.modify(|_, w|
            w.afrh13().af11());

        gpioc.afrl.modify(|_, w|
            w.afrl1().af11()
             .afrl4().af11()
             .afrl5().af11());

        gpiod.afrh.modify(|_, w|
            w.afrh8().af7()
             .afrh9().af7());

        gpiog.afrh.modify(|_, w|
            w.afrh11().af11()
             .afrh13().af11());
    });
}

fn rcc_init() {
    cortex_m::interrupt::free(|cs| {
        let rcc = RCC.borrow(cs);
        let pwr = PWR.borrow(cs);
        let flash = FLASH.borrow(cs);

        rcc.apb1enr.modify(|_, w| w.pwren().enabled());

        // VOS_SCALE2
        unsafe { pwr.cr1.write(|w| w.bits(0x00008000)) }

        // Ensure HSI is on and stable
        rcc.cr.modify(|_, w| w.hsion().set_bit());
        while rcc.cr.read().hsion().is_bit_clear() {}

        // Set to HSI
        rcc.cfgr.modify(|_, w| w.sw().hsi());
        while !rcc.cfgr.read().sws().is_hsi() {}

        // Clear register to reset value
        rcc.cr.write(|w| w.hsion().set_bit());
        rcc.cfgr.write(|w| unsafe { w.bits(0) });

        // Activate PLL
        rcc.pllcfgr.write(|w| unsafe { w.pllq().bits(4).pllsrc().hsi().pllp().div2().plln().bits(168).pllm().bits(8) });
        rcc.cr.modify(|_, w| w.pllon().set_bit());

        // Other clock settings
        rcc.cfgr.modify(|_, w| w.ppre2().div2().ppre1().div4().hpre().div1());

        // TODO DCKFGR2

        // Flash setup
        flash.acr.write(|w| unsafe { w.arten().set_bit().prften().set_bit().latency().bits(5) });

        // Swap to PLL
        rcc.cfgr.modify(|_, w| w.sw().pll());
        while !rcc.cfgr.read().sws().is_pll() {}

    });
}

fn rcc_reset() {
    // Reset peripherals in use
    cortex_m::interrupt::free(|cs| {
        let rcc = RCC.borrow(cs);
        rcc.ahb1rstr.write(|w|
            w.gpioarst().reset()
             .gpiobrst().reset()
             .gpiocrst().reset()
             .gpiodrst().reset()
             .gpiogrst().reset()
             .ethmacrst().reset()
             .dma1rst().reset()
             .dma2rst().reset()
        );
        rcc.apb1rstr.write(|w|
            w.uart3rst().reset()
             .tim5rst().reset()
        );
        rcc.ahb1rstr.write(|w| unsafe { w.bits(0)});
        rcc.apb1rstr.write(|w| unsafe { w.bits(0)});
    });
}

fn rcc_start() {
    // Enable GPIO and ethernet clocks
    cortex_m::interrupt::free(|cs| {
        let rcc = RCC.borrow(cs);
        rcc.ahb1enr.modify(|_, w|
            w.gpioaen().enabled()
             .gpioben().enabled()
             .gpiocen().enabled()
             .gpioden().enabled()
             .gpiogen().enabled()
             .ethmacrxen().enabled()
             .ethmactxen().enabled()
             .ethmacen().enabled()
             .dma1en().enabled()
             .dma2en().enabled()
        );
        rcc.apb2enr.modify(|_, w|
            w.syscfgen().enabled()
        );
        rcc.apb1enr.modify(|_, w|
            w.usart3en().enabled()
             .tim5en().enabled()
        );
    });
}

fn usart_init() {
    cortex_m::interrupt::free(|cs| {
        let usart3 = USART3.borrow(cs);
        usart3.cr1.modify(|_, w|
            w.te().set_bit()
             .re().set_bit()
        );
        usart3.cr3.modify(|_, w|
            w.dmat().set_bit()
        );
        usart3.brr.write(|w| unsafe { w.bits(84) });
        usart3.cr1.modify(|_, w| w.ue().set_bit());
    });
}

fn usart_send_char(data: u32) {
    cortex_m::interrupt::free(|cs| {
        let usart3 = USART3.borrow(cs);
        while usart3.isr.read().txe().is_bit_clear() {}
        usart3.tdr.write(|w| unsafe { w.bits(data) });
    });
}

fn usart_send_string(data: &str) {
    for c in data.as_bytes() {
        usart_send_char(*c as u32);
    }
    usart_wait();
}

fn usart_send_by_dma(data: &[u8]) {
    cortex_m::interrupt::free(|cs| {
        let usart3 = USART3.borrow(cs);
        let dma1 = DMA1.borrow(cs);
        dma1.s3cr.write(|w| w.en().disabled());
        dma1.lifcr.write(|w|
            w.ctcif3().clear()
             .chtif3().clear()
             .cteif3().clear()
             .cdmeif3().clear()
             .cfeif3().clear()
        );
        dma1.s3cr.write(|w| unsafe {
            w.chsel().bits(4)
             .msize().byte()
             .psize().byte()
             .pinc().fixed()
             .minc().incremented()
             .dir().memory_to_peripheral()
             .pfctrl().dma()
        });
        dma1.s3ndtr.write(|w| unsafe { w.ndt().bits(data.len() as u16) });
        dma1.s3par.write(|w| unsafe { w.bits(&usart3.tdr as *const _ as u32) });
        dma1.s3m0ar.write(|w| unsafe { w.bits(data.as_ptr() as u32) });
        usart3.icr.write(|w| w.tccf().set_bit());
        dma1.s3cr.modify(|_, w| w.en().enabled());
    });
    usart_wait();
}

fn usart_wait() {
    cortex_m::interrupt::free(|cs| {
        let usart3 = USART3.borrow(cs);
        while usart3.isr.read().tc().is_bit_clear() {}
        usart3.icr.write(|w| w.tccf().set_bit());
    });
}

fn timer_init() {
    cortex_m::interrupt::free(|cs| {
        let tim5 = TIM5.borrow(cs);
        tim5.cr1.modify(|_, w| w.cen().clear_bit());
        tim5.psc.write(|w| unsafe { w.psc().bits(42000 - 1) });
        tim5.cnt.write(|w| unsafe { w.bits(0) });
        tim5.arr.write(|w| unsafe { w.bits(0xffff_ffff) });
        tim5.egr.write(|w| w.ug().set_bit());
        tim5.cr1.modify(|_, w| w.cen().set_bit());
    });
}

#[inline]
fn timer_time() -> u32 {
    cortex_m::interrupt::free(|cs| {
        let tim5 = TIM5.borrow(cs);
        tim5.cnt.read().bits()
    })
}

fn mac_init() {
    cortex_m::interrupt::free(|cs| {
        let syscfg = SYSCFG.borrow(cs);
        let eth_mac = ETHERNET_MAC.borrow(cs);
        let eth_dma = ETHERNET_DMA.borrow(cs);

        // Set to RMII mode
        syscfg.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());

        // Reset ETH DMA
        eth_dma.dmabmr.modify(|_, w| w.sr().set_bit());
        while eth_dma.dmabmr.read().sr().is_bit_set() {}

        // Set MAC address 56:54:9f:08:87:1d
        eth_mac.maca0lr.write(|w| unsafe { w.bits(0x08_9f_54_56) });
        eth_mac.maca0hr.write(|w| unsafe { w.bits(0x1d_87) });

        // Enable RX and TX now. Set link speed and duplex at link-up event.
        // We also enable stripping the Ethernet CRCs as we won't use them.
        eth_mac.maccr.write(|w|
            w.re().set_bit()
             .te().set_bit()
             .cstf().set_bit()
        );

        // Unsafe access to static TD/RD and TBUF/RBUF
        unsafe {
            // Set up each TDes in ring mode with associated buffer
            for (td, tdbuf) in TD.iter_mut().zip(TBUF.iter()) {
                td.tdes0 = (1<<29) | (1<<28);
                td.tdes2 = tdbuf as *const _ as u32;
                td.tdes3 = 0;
            }

            // Set up each RDes in ring mode with associated buffer
            for (rd, rdbuf) in RD.iter_mut().zip(RBUF.iter()) {
                rd.rdes0 = 1<<31;
                rd.rdes1 = rdbuf.len() as u32 * 4;
                rd.rdes2 = rdbuf as *const _ as u32;
                rd.rdes3 = 0;
            }

            // Mark the final TDes and RDes as end-of-ring
            TD.last_mut().unwrap().tdes0 |= 1<<21;
            RD.last_mut().unwrap().rdes1 |= 1<<15;

            // Tell the ETH DMA the start of each ring
            eth_dma.dmatdlar.write(|w| w.bits(TD.as_ptr() as u32));
            eth_dma.dmardlar.write(|w| w.bits(RD.as_ptr() as u32));
        }

        // Set DMA bus mode
        eth_dma.dmabmr.write(|w| unsafe {
            w.aab().set_bit()
             .pbl().bits(1)
             .sr().clear_bit()
        });

        // Flush TX FIFO
        eth_dma.dmaomr.write(|w| w.ftf().set_bit());
        while eth_dma.dmaomr.read().ftf().is_bit_set() {}

        // Set operation mode to store-and-forward and start DMA
        eth_dma.dmaomr.write(|w|
            w.rsf().set_bit()
             .tsf().set_bit()
             .st().set_bit()
             .sr().set_bit()
        );
    });
}

fn smi_read(reg: u8) -> u16 {
    let mut result = 0;

    cortex_m::interrupt::free(|cs| {
        let eth_mac = ETHERNET_MAC.borrow(cs);

        // Use PHY address 00000, set register address,
        // set clock to HCLK/102, start read operation
        eth_mac.macmiiar.write(|w| unsafe {
            w.mb().set_bit()
             .cr().bits(4)
             .mr().bits(reg)
        });

        // Wait for read to complete
        while eth_mac.macmiiar.read().mb().is_bit_set() {}

        // Return resulting data
        result = eth_mac.macmiidr.read().td().bits();
    });

    result
}

fn smi_write(reg: u8, val: u16) {
    cortex_m::interrupt::free(|cs| {
        let eth_mac = ETHERNET_MAC.borrow(cs);

        eth_mac.macmiidr.write(|w| unsafe { w.td().bits(val) });
        eth_mac.macmiiar.write(|w| unsafe {
            w.mb().set_bit()
             .mw().set_bit()
             .cr().bits(4)
             .mr().bits(reg)
        });

        while eth_mac.macmiiar.read().mb().is_bit_set() {}
    });
}

fn phy_reset() {
    smi_write(0x00, (1<<15));
    while smi_read(0x00) & (1<<15) == (1<<15) {}
}

fn phy_init() {
    smi_write(0x00, (1<<12));
}

fn phy_poll_link() -> bool {
    let bsr = smi_read(0x01);
    let bcr = smi_read(0x00);
    let lpa = smi_read(0x05);

    // No link if no auto negotiate
    if bcr & (1<<12) == 0 { return false; }
    // No link if link is down
    if bsr & (1<< 2) == 0 { return false; }
    // No link if remote fault
    if bsr & (1<< 4) != 0 { return false; }
    // No link if autonegotiate incomplete
    if bsr & (1<< 5) == 0 { return false; }
    // No link if other side can't do 100Mbps full duplex
    if lpa & (1<< 8) == 0 { return false; }

    // Otherwise we have link
    cortex_m::interrupt::free(|cs| {
        let eth_mac = ETHERNET_MAC.borrow(cs);
        eth_mac.maccr.modify(|_, w| w.fes().set_bit().dm().set_bit());
    });

    true
}

#[inline(never)]
fn cache_test() {
    cortex_m::interrupt::free(|cs| {
        let scb = cortex_m::peripheral::SCB.borrow(cs);
        let cpuid = cortex_m::peripheral::CPUID.borrow(cs);
        scb.disable_dcache(&cpuid);

        let mut x = ['A' as u8, '\r' as u8, '\n' as u8];

        usart_send_string("Operation               Cache  RAM\r\n");
        usart_send_string("----------------------------------\r\n");

        fn step(op: &str, x: &[u8]) {
            usart_send_string(op);
            for _ in 0..27-op.len() {
                usart_send_char('.' as u32);
            }
            usart_wait();
            usart_send_char(x[0] as u32);
            usart_wait();
            for _ in 0..4 {
                usart_send_char(' ' as u32);
            }
            usart_wait();
            usart_send_by_dma(x);
            asm::dsb();
            asm::isb();
        }

        step("Initial", &x);

        x[0] += 1;
        step("Increment", &x);

        scb.enable_dcache(&cpuid);
        step("Enable DCache", &x);

        x[0] += 1;
        step("Increment", &x);

        scb.clean_dcache(&cpuid);
        step("Clean DCache", &x);

        x[0] += 1;
        step("Increment", &x);

        scb.invalidate_dcache(&cpuid);
        step("Invalidate DCache", &x);

        x[0] += 1;
        step("Increment", &x);

        scb.clean_invalidate_dcache(&cpuid);
        step("CleanInvalidate DCache", &x);

        x[0] += 1;
        step("Increment", &x);

        scb.clean_dcache_by_address(0, 32);
        step("Clean DCache wrong addr", &x);

        scb.clean_dcache_by_address(x.as_ptr() as u32, 32);
        step("Clean DCache right addr", &x);

        x[0] += 1;
        step("Increment", &x);

        scb.invalidate_dcache_by_address(0, 32);
        step("Inval DCache wrong addr", &x);

        scb.invalidate_dcache_by_address(x.as_ptr() as u32, 32);
        step("Inval DCache right addr", &x);

        x[0] += 1;
        step("Increment", &x);

        scb.clean_invalidate_dcache_by_address(0, 32);
        step("CI DCache wrong addr", &x);

        scb.clean_invalidate_dcache_by_address(x.as_ptr() as u32, 32);
        step("CI DCache right addr", &x);

        x[0] += 1;
        step("Increment", &x);

        scb.disable_dcache(&cpuid);
        step("Disable DCache", &x);
        asm::bkpt();
    });
}

#[inline(never)]
fn bench_memory() {
    let cntptr = cortex_m::interrupt::free(|cs| {
        let tim5 = TIM5.borrow(cs);
        &tim5.cnt as *const _ as u32
    });
    let start: u32;
    let end: u32;
    unsafe { BENCH_BUF[0] = 0x00FF_FFFF; }
    unsafe { asm!("
        ldr r2, [r4]
        1:
        ldr r1, [r0, #28]
        ldr r1, [r0, #24]
        ldr r1, [r0, #20]
        ldr r1, [r0, #16]
        ldr r1, [r0, #12]
        ldr r1, [r0, #8]
        ldr r1, [r0, #4]
        ldr r1, [r0, #0]
        subs r1, #1
        str r1, [r0, #0]
        str r1, [r0, #4]
        str r1, [r0, #8]
        str r1, [r0, #12]
        str r1, [r0, #16]
        str r1, [r0, #20]
        str r1, [r0, #24]
        str r1, [r0, #28]
        bne 1b
        ldr r3, [r4]
    "
    : "={r2}"(start), "={r3}"(end)
    : "{r0}"(BENCH_BUF.as_mut_ptr() as u32), "{r4}"(cntptr)
    : "r1", "cc", "memory"
    : "volatile"
    ); }
    let duration = end.wrapping_sub(start);
    hprintln!("Memory benchmark took {}ms", duration);
}

#[inline(never)]
fn main() {
    cortex_m::interrupt::free(|cs| {
        let scb = cortex_m::peripheral::SCB.borrow(cs);
        let cpuid = cortex_m::peripheral::CPUID.borrow(cs);
        scb.disable_icache();
        scb.disable_dcache(&cpuid);
    });

    rcc_init();
    rcc_start();
    rcc_reset();
    gpio_init();
    timer_init();
    usart_init();

    hprintln!("\r\nRunning memory benchmark with DCache OFF, ICache ON\r\n");
    cortex_m::interrupt::free(|cs| {
        let scb = cortex_m::peripheral::SCB.borrow(cs);
        let cpuid = cortex_m::peripheral::CPUID.borrow(cs);
        scb.disable_dcache(&cpuid);
        scb.enable_icache();
    });
    bench_memory();
    bench_memory();
    bench_memory();
    bench_memory();

    hprintln!("\r\nRunning memory benchmark with DCache ON, ICache ON\r\n");
    cortex_m::interrupt::free(|cs| {
        let scb = cortex_m::peripheral::SCB.borrow(cs);
        let cpuid = cortex_m::peripheral::CPUID.borrow(cs);
        scb.enable_dcache(&cpuid);
        scb.enable_icache();
    });
    bench_memory();
    bench_memory();
    bench_memory();
    bench_memory();

    hprintln!("\r\nRunning memory benchmark with DCache ON, ICache OFF\r\n");
    cortex_m::interrupt::free(|cs| {
        let scb = cortex_m::peripheral::SCB.borrow(cs);
        let cpuid = cortex_m::peripheral::CPUID.borrow(cs);
        scb.enable_dcache(&cpuid);
        scb.disable_icache();
    });
    bench_memory();
    bench_memory();
    bench_memory();
    bench_memory();

    hprintln!("\r\nRunning memory benchmark with DCache OFF, ICache OFF\r\n");
    cortex_m::interrupt::free(|cs| {
        let scb = cortex_m::peripheral::SCB.borrow(cs);
        let cpuid = cortex_m::peripheral::CPUID.borrow(cs);
        scb.disable_dcache(&cpuid);
        scb.disable_icache();
    });
    bench_memory();
    bench_memory();
    bench_memory();
    bench_memory();

    usart_send_string("\r\n\r\nRunning cache test...\r\n");
    cache_test();

    usart_send_string("\r\n\r\nInitialising network...\r\n");

    phy_reset();
    phy_init();
    mac_init();

    let mut arpcache_storage = [Default::default(); 8];
    let mut arpcache = SliceArpCache::new(&mut arpcache_storage[..]);
    let mut ethdev = EthernetDevice {};
    let ethaddr = EthernetAddress::from_bytes(&[0x56, 0x54, 0x9f, 0x08, 0x87, 0x1d]);
    let mut ipaddrs = [IpAddress::v4(192, 168, 2, 202)];
    let mut ethiface = EthernetInterface::new(
        &mut ethdev, &mut arpcache as &mut ArpCache, ethaddr, &mut ipaddrs[..]);

    let mut tcp_rx_buf_storage = [0u8; 128];
    let mut tcp_tx_buf_storage = [0u8; 128];
    let tcp_rx_buf = TcpSocketBuffer::new(&mut tcp_rx_buf_storage[..]);
    let tcp_tx_buf = TcpSocketBuffer::new(&mut tcp_tx_buf_storage[..]);
    let tcp_socket = TcpSocket::new(tcp_rx_buf, tcp_tx_buf);

    let mut sockets_storage = [None, None, None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let tcp_handle = sockets.add(tcp_socket);

    usart_send_string("Waiting for link...\r\n");
    while !phy_poll_link() { }
    usart_send_string("Link established.\r\n");

    loop {
        {
            let socket: &mut TcpSocket = sockets.get_mut(tcp_handle).as_socket();
            if !socket.is_open() {
                socket.listen(6969).unwrap();
            }
            if socket.can_send() {
                let data = b"hello world\n";
                usart_send_string("Sending TCP data\r\n");
                socket.send_slice(data).unwrap();
                usart_send_string("Closing socket\r\n");
                socket.close();
            }
        }

        let time_ms = timer_time() as u64;
        match ethiface.poll(&mut sockets, time_ms) {
            Ok(()) | Err(smoltcp::Error::Exhausted) => (),
            Err(_) => usart_send_string("Network error\r\n"),
        }
    }
}

#[allow(dead_code)]
#[used]
#[link_section = ".rodata.interrupts"]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    asm::bkpt();
}
