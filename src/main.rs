// #![no_main]
// #![no_std]

// use core::cell::RefCell;

// use panic_halt as _;

// use cortex_m::{asm, interrupt::Mutex, peripheral::NVIC};
// use cortex_m_rt::entry;

// use stm32f3xx_hal::{
//     gpio::{self, Edge, Input, Output, PushPull},
//     interrupt, pac,
//     prelude::*,
// };

// type LedPin = gpio::PA5<Output<PushPull>>;
// static LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));

// type ButtonPin = gpio::PC13<Input>;
// static BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));

// // When the user button is pressed. The north LED will toggle.
// #[entry]
// fn main() -> ! {
//     // Getting access to registers we will need for configuration.
//     let device_peripherals = pac::Peripherals::take().unwrap();
//     let mut rcc = device_peripherals.RCC.constrain();
//     let mut syscfg = device_peripherals.SYSCFG.constrain(&mut rcc.apb2);
//     let mut exti = device_peripherals.EXTI;
//     let mut gpioa = device_peripherals.GPIOA.split(&mut rcc.ahb);
//     let mut gpioc = device_peripherals.GPIOC.split(&mut rcc.ahb);

//     let mut led = gpioa
//         .pa5
//         .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
//     // Turn the led on so we know the configuration step occurred.
//     led.toggle().expect("unable to toggle led in configuration");

//     // Move the ownership of the led to the global LED
//     cortex_m::interrupt::free(|cs| *LED.borrow(cs).borrow_mut() = Some(led));

//     // Configuring the user button to trigger an interrupt when the button is pressed.
//     let mut user_button = gpioc
//         .pc13
//         .into_pull_down_input(&mut gpioc.moder, &mut gpioc.pupdr);
//     syscfg.select_exti_interrupt_source(&user_button);
//     user_button.trigger_on_edge(&mut exti, Edge::Rising);
//     user_button.enable_interrupt(&mut exti);
//     let interrupt_num = user_button.interrupt(); // hal::pac::Interrupt::EXTI0

//     // Moving ownership to the global BUTTON so we can clear the interrupt pending bit.
//     cortex_m::interrupt::free(|cs| *BUTTON.borrow(cs).borrow_mut() = Some(user_button));

//     unsafe { NVIC::unmask(interrupt_num) };

//     loop {
//         asm::wfi();
//     }
// }

// Button Pressed interrupt.
// The exti# maps to the pin number that is being used as an external interrupt.
// See page 295 of the stm32f303 reference manual for proof:
// http://www.st.com/resource/en/reference_manual/dm00043574.pdf
//
// This may be called more than once per button press from the user since the button may not be debounced.



//! Example of configuring spi.
//! Target board: STM32F3DISCOVERY
#![no_std]
#![no_main]

use panic_halt as _;

use core::cell::RefCell;

use stm32f3xx_hal as hal;

use cortex_m::{asm, interrupt::Mutex, peripheral::NVIC};
use cortex_m_rt::entry;

use hal::pac;
use hal::prelude::*;
use hal::spi::Spi;
use hal::interrupt;
use hal::gpio::{self, Edge, Input, Output, PushPull};

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::{spi::FullDuplex, spi::Mode, spi::Phase, spi::Polarity};
use embedded_hal::blocking::spi::Transfer;
use max7219_dot_matrix::{Command, MAX7219};
use stm32f3xx_hal::pac::SPI2;

//static MY_SPI: Mutex<RefCell<Option<stm32f3xx_hal::spi::Spi<SPI2, >>>>> = Mutex::new(RefCell::new(None));

static mut IS_CLICKED: bool = false;
static mut RIGHT: u8 = 0;

type ButtonPin = gpio::PC13<Input>;
static BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut syscfg = dp.SYSCFG.constrain(&mut rcc.apb2);
    let mut exti = dp.EXTI;
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);


    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .freeze(&mut flash.acr);

    // Configure pins for SPI
    let sck = gpiob
        .pb13
        .into_af5_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let miso = gpiob
        .pb14
        .into_af5_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let mosi = gpiob
        .pb15
        .into_af5_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let mut cs = gpiob
        .pb12
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    let mut spi = Spi::new(dp.SPI2, (sck, miso, mosi), 3.MHz(), clocks, &mut rcc.apb1);


    let mut user_button = gpioc
        .pc13
        .into_pull_down_input(&mut gpioc.moder, &mut gpioc.pupdr);

    syscfg.select_exti_interrupt_source(&user_button);
    user_button.trigger_on_edge(&mut exti, Edge::Rising);
    user_button.enable_interrupt(&mut exti);
    let interrupt_num = user_button.interrupt();

    cortex_m::interrupt::free(|cs| *BUTTON.borrow(cs).borrow_mut() = Some(user_button));

    unsafe { NVIC::unmask(interrupt_num) };

    // max 7219 setup for 20 chips
    let mut max7219 = MAX7219::new(&mut cs, 1);

    //demo_print_string(&mut max7219, &mut spi).unwrap();
    // unsafe {
    //     loop {
    //         if IS_CLICKED {
    //             demo_print_string(&mut max7219, &mut spi).unwrap();
    //         }
    //     }
    // }

    demo_print_string(&mut max7219, &mut spi).unwrap();

    loop {}
}

fn demo_print_string<SpiError, PinError, CS>(
    max7219: &mut MAX7219<CS>,
    spi: &mut dyn Transfer<u8, Error = SpiError>,
) -> Result<(), max7219_dot_matrix::Error<SpiError, PinError>>
    where
        CS: OutputPin<Error = PinError>,
{
    max7219.write_command_all(spi, Command::OnOff, 0)?;
    max7219.write_command_all(spi, Command::ScanLimit, 7)?;
    max7219.write_command_all(spi, Command::DecodeMode, 0)?;
    max7219.write_command_all(spi, Command::DisplayTest, 0)?;
    max7219.write_command_all(spi, Command::Intensity, 1)?;
    max7219.clear_all(spi)?;
    max7219.write_command_all(spi, Command::OnOff, 1)?;

    let arr: [u8; 7] = [
        0b00000000,
        0b00000000,
        0b00000000,
        0b00000000,
        0b00000000,
        0b00000000,
        0b10000000,
    ];

    let left = 0;
    let right = 7;

    let mut count = 0;

    for i in arr {
        max7219.write_line_raw(spi, count, &[i]);
        count = count + 1;
    }
    unsafe {
        loop {
            if IS_CLICKED {
                max7219.write_line_raw(spi, 6, &[0b10000000 >> RIGHT]);
                IS_CLICKED = false;
            }
        } 
    }
    
    Ok(())
}

#[interrupt]
fn EXTI15_10() {
    unsafe {
        IS_CLICKED = true; 
        RIGHT += 1;
    }

    cortex_m::interrupt::free(|cs| {


        // Clear the interrupt pending bit so we don't infinitely call this routine
        BUTTON
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .clear_interrupt();
    })
}