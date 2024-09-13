use std::{env::var, fmt::Display, fs::File, io::Write, sync::Mutex};

#[macro_export]
macro_rules! log {
    ($target: expr, $($arg:tt)*) => {
        $target.log(format_args!($($arg)*));
    };
}

pub enum LogTarget {
    Stdout,
    Stderr,
    File(Mutex<File>),
}

impl LogTarget {
    pub fn parse(str: &str) -> Option<LogTarget> {
        if str.trim().is_empty() {
            return None;
        }

        Some(match str {
            "stdout" => LogTarget::Stdout,
            "stderr" => LogTarget::Stderr,
            str => LogTarget::File(Mutex::new(File::open(str).expect("fail to open log file"))),
        })
    }
}

pub struct Logger {
    target: Option<LogTarget>,
}

impl Logger {
    pub fn new(name: &str) -> Self {
        Self {
            target: LogTarget::parse(&var(format!("LOG_{name}")).unwrap_or_default()),
        }
    }

    pub fn log(&self, value: impl Display) {
        match &self.target {
            Some(LogTarget::Stdout) => println!("{}", value),
            Some(LogTarget::Stderr) => eprintln!("{}", value),
            Some(LogTarget::File(file)) => {
                let mut file = file.lock().expect("mutex lock failure");
                writeln!(&mut file, "{}", value).expect("write failed");
            }
            None => {}
        }
    }

    pub fn enabled(&self) -> bool {
        self.target.is_some()
    }
}
