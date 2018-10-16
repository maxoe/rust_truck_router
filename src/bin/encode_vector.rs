extern crate stud_rust_base;
use stud_rust_base::io::*;
use std::env;

fn main() {
    let mut args = env::args();
    args.next();

    match &args.collect::<Vec<String>>()[..] {
        [data_type, output] => {
            match data_type.as_ref() {
                "i8" => parse_input::<i8>().write_to(output).expect("Failed to write to output"),
                "u8" => parse_input::<u8>().write_to(output).expect("Failed to write to output"),
                "i16" => parse_input::<i16>().write_to(output).expect("Failed to write to output"),
                "u16" => parse_input::<u16>().write_to(output).expect("Failed to write to output"),
                "i32" => parse_input::<i32>().write_to(output).expect("Failed to write to output"),
                "u32" => parse_input::<u32>().write_to(output).expect("Failed to write to output"),
                "i64" => parse_input::<i64>().write_to(output).expect("Failed to write to output"),
                "u64" => parse_input::<u64>().write_to(output).expect("Failed to write to output"),
                "f32" => parse_input::<f32>().write_to(output).expect("Failed to write to output"),
                "f64" => parse_input::<f64>().write_to(output).expect("Failed to write to output"),
                "int8" => parse_input::<i8>().write_to(output).expect("Failed to write to output"),
                "uint8" => parse_input::<u8>().write_to(output).expect("Failed to write to output"),
                "int16" => parse_input::<i16>().write_to(output).expect("Failed to write to output"),
                "uint16" => parse_input::<u16>().write_to(output).expect("Failed to write to output"),
                "int32" => parse_input::<i32>().write_to(output).expect("Failed to write to output"),
                "uint32" => parse_input::<u32>().write_to(output).expect("Failed to write to output"),
                "int64" => parse_input::<i64>().write_to(output).expect("Failed to write to output"),
                "uint64" => parse_input::<u64>().write_to(output).expect("Failed to write to output"),
                "float32" => parse_input::<f32>().write_to(output).expect("Failed to write to output"),
                "float64" => parse_input::<f64>().write_to(output).expect("Failed to write to output"),
                _ => {
                    print_usage();
                    panic!("Unknown data_type {}", data_type);
                }
            };
        },
        _ => {
            print_usage();
            panic!("Invalid input")
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

use std::{
    str::FromStr,
    fmt::Debug,
};

fn parse_input<T>() -> Vec<T> where
    T: FromStr,
    <T as FromStr>::Err: Debug
{
    use std::io::{BufRead, stdin};

    let mut values = Vec::new();

    let stdin = stdin();
    for line in stdin.lock().lines() {
        values.push(line.unwrap().parse::<T>().unwrap())
    }

    values
}
