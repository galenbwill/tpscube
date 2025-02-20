use crate::font::{FontSize, LabelFontSize};
use crate::theme::Theme;
use chrono::{DateTime, Local};
use egui::{widgets::Label, Color32, Pos2, Response, Sense, Stroke, Ui, Vec2};
use tpscube_core::Move;

const MIN_SCRAMBLE_LINES: usize = 2;
const MAX_SCRAMBLE_LINES: usize = 5;

pub trait CustomWidgets {
    fn header_label(&mut self, icon: &str, text: &str, landscape: bool, active: bool) -> Response;
    fn mode_label(&mut self, text: &str, active: bool) -> Response;
    fn section(&mut self, text: &str);
    fn section_separator(&mut self);
}

pub fn solve_time_string(time: u32) -> String {
    let time = (time + 5) / 10;
    if time > 6000 {
        format!(
            "{}:{:02}.{:02}",
            time / 6000,
            (time % 6000) / 100,
            time % 100
        )
    } else {
        format!("{}.{:02}", time / 100, time % 100)
    }
}

pub fn solve_time_string_ms(time: u32) -> String {
    if time > 60000 {
        format!(
            "{}:{:02}.{:03}",
            time / 60000,
            (time % 60000) / 1000,
            time % 1000
        )
    } else {
        format!("{}.{:03}", time / 1000, time % 1000)
    }
}

pub fn solve_time_short_string(time: u32) -> String {
    let time = time / 100;
    if time > 600 {
        format!("{}:{:02}.{}", time / 600, (time % 600) / 10, time % 10)
    } else {
        format!("{}.{}", time / 10, time % 10)
    }
}

pub fn short_day_string(time: &DateTime<Local>) -> String {
    let now = Local::now();
    let current_day = now.date();
    let target_day = time.date();
    let days = (current_day - target_day).num_days();
    match days {
        0..=364 => format!(
            "{} {}",
            target_day.format("%b"),
            target_day.format("%e").to_string().trim(),
        ),
        _ => format!(
            "{} {}",
            target_day.format("%b"),
            target_day.format("%e, %Y").to_string().trim(),
        ),
    }
}

pub fn date_string(time: &DateTime<Local>) -> String {
    let now = Local::now();
    let current_day = now.date();
    let target_day = time.date();
    let days = (current_day - target_day).num_days();
    match days {
        0 => format!(
            "Today at {}",
            time.time().format("%l:%M %P").to_string().trim()
        ),
        1 => format!(
            "Yesterday at {}",
            time.time().format("%l:%M %P").to_string().trim()
        ),
        2..=6 => format!(
            "{} at {}",
            target_day.format("%A"),
            time.time().format("%l:%M %P").to_string().trim()
        ),
        7..=364 => format!(
            "{} {} at {}",
            target_day.format("%B"),
            target_day.format("%e").to_string().trim(),
            time.time().format("%l:%M %P").to_string().trim()
        ),
        _ => format!(
            "{} {} at {}",
            target_day.format("%B"),
            target_day.format("%e, %Y").to_string().trim(),
            time.time().format("%l:%M %P").to_string().trim()
        ),
    }
}

fn scramble_lines(scramble: &[Move], line_count: usize) -> Vec<Vec<Move>> {
    let per_line = (scramble.len() + line_count - 1) / line_count;
    let mut lines = Vec::new();
    for chunks in scramble.chunks(per_line) {
        lines.push(chunks.to_vec());
    }
    lines
}

pub fn fit_scramble(ui: &Ui, font: FontSize, scramble: &[Move], width: f32) -> Vec<Vec<Move>> {
    for line_count in MIN_SCRAMBLE_LINES..MAX_SCRAMBLE_LINES {
        let lines = scramble_lines(scramble, line_count);
        if !lines.iter().any(|line| {
            ui.fonts()
                .layout_single_line(
                    font.into(),
                    line.iter()
                        .map(|mv| mv.to_string())
                        .collect::<Vec<String>>()
                        .join("  "),
                )
                .size
                .x
                > width
        }) {
            return lines;
        }
    }
    scramble_lines(scramble, MAX_SCRAMBLE_LINES)
}

pub fn color_for_step_index(idx: usize) -> Color32 {
    match idx % 4 {
        0 => Theme::Red.into(),
        1 => Theme::Blue.into(),
        2 => Theme::Yellow.into(),
        3 => Theme::Green.into(),
        4 => Theme::Cyan.into(),
        5 => Theme::Magenta.into(),
        _ => unreachable!(),
    }
}

pub fn color_for_recognition_step_index(idx: usize) -> Color32 {
    // Don't make darker with alpha channel directly, as WebGL
    // does not blend it properly. Compute linear space mulitplier
    // and set alpha to opaque.
    let color = color_for_step_index(idx).linear_multiply(0.4);
    Color32::from_rgb(color.r(), color.g(), color.b())
}

impl CustomWidgets for Ui {
    fn header_label(&mut self, icon: &str, text: &str, landscape: bool, active: bool) -> Response {
        if landscape {
            self.add(
                if active {
                    Label::new(format!("{}  {}", icon, text)).text_color(Theme::Green)
                } else {
                    Label::new(format!("{}  {}", icon, text))
                }
                .sense(Sense::click()),
            )
        } else {
            // In portrait mode, display icon with small text below
            let icon_galley = self
                .fonts()
                .layout_single_line(FontSize::Normal.into(), icon.into());
            let text_galley = self
                .fonts()
                .layout_single_line(FontSize::Small.into(), text.into());

            let (response, painter) = self.allocate_painter(
                Vec2::new(text_galley.size.x, icon_galley.size.y + text_galley.size.y),
                Sense::click(),
            );

            let icon_height = icon_galley.size.y;
            let color = if active {
                Theme::Green.into()
            } else if response.hovered() {
                Theme::Content.into()
            } else {
                Theme::Disabled.into()
            };
            painter.galley(
                Pos2::new(
                    response.rect.center().x - icon_galley.size.x / 2.0,
                    response.rect.top(),
                ),
                icon_galley,
                color,
            );

            painter.galley(
                Pos2::new(response.rect.left(), response.rect.top() + icon_height),
                text_galley,
                color,
            );

            response
        }
    }

    fn mode_label(&mut self, text: &str, active: bool) -> Response {
        self.add(
            if active {
                Label::new(format!("{}", text))
                    .text_color(Theme::Green)
                    .wrap(false)
            } else {
                Label::new(format!("{}", text)).wrap(false)
            }
            .sense(Sense::click()),
        )
    }

    fn section(&mut self, text: &str) {
        self.add(
            Label::new(text)
                .font_size(FontSize::Section)
                .text_color(Theme::Blue),
        );
        self.section_separator();
    }

    fn section_separator(&mut self) {
        self.scope(|ui| {
            ui.style_mut().visuals.widgets.noninteractive.bg_stroke = Stroke {
                width: 1.0,
                color: Theme::DarkBlue.into(),
            };
            ui.separator();
        });
    }
}
