use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::time::Duration;

use gpui::{
    div, prelude::*, px, rgb, size, App, Application, Bounds, Context, SharedString, Window,
    WindowBounds, WindowOptions,
};
 
struct HelloWorld {
    text: SharedString,
}
 
impl Render for HelloWorld {
    fn render(&mut self, _window: &mut Window, _cx: &mut Context<Self>) -> impl IntoElement {
        div()
            .flex()
            .flex_col()
            .gap_3()
            .bg(rgb(0x505050))
            .size(px(500.0))
            .justify_center()
            .items_center()
            .shadow_lg()
            .border_1()
            .border_color(rgb(0x0000ff))
            .text_xl()
            .text_color(rgb(0xffffff))
            .child(format!("Hello, {}!", &self.text))
            .child(
                div()
                    .flex()
                    .gap_2()
                    .child(div().size_8().bg(gpui::red()))
                    .child(div().size_8().bg(gpui::green()))
                    .child(div().size_8().bg(gpui::blue()))
                    .child(div().size_8().bg(gpui::yellow()))
                    .child(div().size_8().bg(gpui::black()))
                    .child(div().size_8().bg(gpui::white())),
            )
    }
}
 
// Must match dump_header_t / capture_t in candy-bar/src/twai_sniffer.c.
const HEADER_LEN: usize = 8;
const CAPTURE_LEN: usize = 16;

/// Pulls complete dumps out of `pending` and appends their captures to `out`.
/// The stream is back-to-back dumps: an 8-byte header ("CNDY" + u32 count)
/// followed by `count` captures.
fn drain_dumps(pending: &mut Vec<u8>, out: &mut File) {
    loop {
        if pending.len() < HEADER_LEN {
            return;
        }
        let count = u32::from_le_bytes(pending[4..8].try_into().unwrap()) as usize;
        let end = HEADER_LEN + count * CAPTURE_LEN;
        if pending.len() < end {
            return;
        }
        out.write_all(&pending[HEADER_LEN..end]).unwrap();
        pending.drain(..end);
    }
}

fn main() {
    for p in serialport::available_ports().unwrap() {
        println!("{}", p.port_name); // on macOS: /dev/cu.usbserial-XXXX or /dev/cu.usbmodemXXXX
    }

    let mut port = serialport::new("/dev/cu.usbmodem101", 115_200)
        .timeout(Duration::from_millis(500))
        .open().unwrap();

    let mut out = OpenOptions::new()
        .create(true)
        .append(true)
        .open("captures.bin")
        .unwrap();
    let mut pending = Vec::new();
    loop {
        let mut buf = [0u8; 1024];
        let n = match port.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            // The sniffer only sends once a second, so quiet gaps are normal.
            Err(e) if e.kind() == io::ErrorKind::TimedOut => continue,
            Err(e) => panic!("serial read failed: {e}"),
        };
        pending.extend_from_slice(&buf[..n]);
        drain_dumps(&mut pending, &mut out);
    }
    
    Application::new().run(|cx: &mut App| {
        let bounds = Bounds::centered(None, size(px(500.), px(500.0)), cx);
        cx.open_window(
            WindowOptions {
                window_bounds: Some(WindowBounds::Windowed(bounds)),
                ..Default::default()
            },
            |_, cx| {
                cx.new(|_| HelloWorld {
                    text: "World".into(),
                })
            },
        )
        .unwrap();
    });
}