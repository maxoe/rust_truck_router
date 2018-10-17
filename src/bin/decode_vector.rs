extern crate stud_rust_base;
use stud_rust_base::io::*;
use std::env;

fn main() {
    let mut args = env::args();
    args.next();

    match &args.collect::<Vec<String>>()[..] {
        [data_type, input] => {
            match data_type.as_ref() {
                "i8" => print_values(Vec::<i8>::load_from(input).expect("Failed to read from input")),
                "u8" => print_values(Vec::<u8>::load_from(input).expect("Failed to read from input")),
                "i16" => print_values(Vec::<i16>::load_from(input).expect("Failed to read from input")),
                "u16" => print_values(Vec::<u16>::load_from(input).expect("Failed to read from input")),
                "i32" => print_values(Vec::<i32>::load_from(input).expect("Failed to read from input")),
                "u32" => print_values(Vec::<u32>::load_from(input).expect("Failed to read from input")),
                "i64" => print_values(Vec::<i64>::load_from(input).expect("Failed to read from input")),
                "u64" => print_values(Vec::<u64>::load_from(input).expect("Failed to read from input")),
                "f32" => print_values(Vec::<f32>::load_from(input).expect("Failed to read from input")),
                "f64" => print_values(Vec::<f64>::load_from(input).expect("Failed to read from input")),
                "int8" => print_values(Vec::<i8>::load_from(input).expect("Failed to read from input")),
                "uint8" => print_values(Vec::<u8>::load_from(input).expect("Failed to read from input")),
                "int16" => print_values(Vec::<i16>::load_from(input).expect("Failed to read from input")),
                "uint16" => print_values(Vec::<u16>::load_from(input).expect("Failed to read from input")),
                "int32" => print_values(Vec::<i32>::load_from(input).expect("Failed to read from input")),
                "uint32" => print_values(Vec::<u32>::load_from(input).expect("Failed to read from input")),
                "int64" => print_values(Vec::<i64>::load_from(input).expect("Failed to read from input")),
                "uint64" => print_values(Vec::<u64>::load_from(input).expect("Failed to read from input")),
                "float32" => print_values(Vec::<f32>::load_from(input).expect("Failed to read from input")),
                "float64" => print_values(Vec::<f64>::load_from(input).expect("Failed to read from input")),
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
    eprintln!("Usage: decode_vector data_type input_vector_file

Reads binary data from input_vector_file and writes the data to the standard output. data_type can be one of
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

use std::fmt::Display;

fn print_values<T>(values: Vec<T>) where
    T: Display
{
    for v in values {
        println!("{}", v);
    }
}
