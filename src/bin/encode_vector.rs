use stud_rust_base::{io::*, cli::CliErr};
use std::{env, error::Error};

fn main() -> Result<(), Box<dyn Error>> {
    match &env::args().skip(1).collect::<Vec<String>>()[..] {
        [data_type, output] => {
            match data_type.as_ref() {
                "i8" => { parse_input::<i8>()?.write_to(output)?; Ok(()) },
                "u8" => { parse_input::<u8>()?.write_to(output)?; Ok(()) },
                "i16" => { parse_input::<i16>()?.write_to(output)?; Ok(()) },
                "u16" => { parse_input::<u16>()?.write_to(output)?; Ok(()) },
                "i32" => { parse_input::<i32>()?.write_to(output)?; Ok(()) },
                "u32" => { parse_input::<u32>()?.write_to(output)?; Ok(()) },
                "i64" => { parse_input::<i64>()?.write_to(output)?; Ok(()) },
                "u64" => { parse_input::<u64>()?.write_to(output)?; Ok(()) },
                "f32" => { parse_input::<f32>()?.write_to(output)?; Ok(()) },
                "f64" => { parse_input::<f64>()?.write_to(output)?; Ok(()) },
                "int8" => { parse_input::<i8>()?.write_to(output)?; Ok(()) },
                "uint8" => { parse_input::<u8>()?.write_to(output)?; Ok(()) },
                "int16" => { parse_input::<i16>()?.write_to(output)?; Ok(()) },
                "uint16" => { parse_input::<u16>()?.write_to(output)?; Ok(()) },
                "int32" => { parse_input::<i32>()?.write_to(output)?; Ok(()) },
                "uint32" => { parse_input::<u32>()?.write_to(output)?; Ok(()) },
                "int64" => { parse_input::<i64>()?.write_to(output)?; Ok(()) },
                "uint64" => { parse_input::<u64>()?.write_to(output)?; Ok(()) },
                "float32" => { parse_input::<f32>()?.write_to(output)?; Ok(()) },
                "float64" => { parse_input::<f64>()?.write_to(output)?; Ok(()) },
                _ => {
                    print_usage();
                    Err(Box::new(CliErr("Invalid data type")))
                }
            }
        },
        _ => {
            print_usage();
            Err(Box::new(CliErr("Invalid arguments")))
        },
    }
}

fn print_usage() {
    eprintln!("Usage: encode_vector data_type output_vector_file

Reads textual data from the standard input and writes it in a binary format to output_vector_file. The input data should be one data element per line. The data is only written once an end of file is encountered on the input. data_type can be one of
* i8
* u8
* i16
* u16
* i32
* u32
* i64
* u64
* f32
* f64

");
}

use std::str::FromStr;

fn parse_input<T>() -> Result<Vec<T>, Box<dyn Error>> where
    T: FromStr,
    <T as FromStr>::Err: Error + 'static
{
    use std::io::{BufRead, stdin};

    let mut values = Vec::new();

    let stdin = stdin();
    for line in stdin.lock().lines() {
        values.push(line.unwrap().parse::<T>()?)
    }

    Ok(values)
}
