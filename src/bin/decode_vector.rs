extern crate stud_rust_base;
use stud_rust_base::{io::*, cli::CliErr};
use std::{env, error::Error};

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args();
    args.next();

    match &args.collect::<Vec<String>>()[..] {
        [data_type, input] => {
            match data_type.as_ref() {
                "i8" => { print_values(Vec::<i8>::load_from(input)?); Ok(()) },
                "u8" => { print_values(Vec::<u8>::load_from(input)?); Ok(()) },
                "i16" => { print_values(Vec::<i16>::load_from(input)?); Ok(()) },
                "u16" => { print_values(Vec::<u16>::load_from(input)?); Ok(()) },
                "i32" => { print_values(Vec::<i32>::load_from(input)?); Ok(()) },
                "u32" => { print_values(Vec::<u32>::load_from(input)?); Ok(()) },
                "i64" => { print_values(Vec::<i64>::load_from(input)?); Ok(()) },
                "u64" => { print_values(Vec::<u64>::load_from(input)?); Ok(()) },
                "f32" => { print_values(Vec::<f32>::load_from(input)?); Ok(()) },
                "f64" => { print_values(Vec::<f64>::load_from(input)?); Ok(()) },
                "int8" => { print_values(Vec::<i8>::load_from(input)?); Ok(()) },
                "uint8" => { print_values(Vec::<u8>::load_from(input)?); Ok(()) },
                "int16" => { print_values(Vec::<i16>::load_from(input)?); Ok(()) },
                "uint16" => { print_values(Vec::<u16>::load_from(input)?); Ok(()) },
                "int32" => { print_values(Vec::<i32>::load_from(input)?); Ok(()) },
                "uint32" => { print_values(Vec::<u32>::load_from(input)?); Ok(()) },
                "int64" => { print_values(Vec::<i64>::load_from(input)?); Ok(()) },
                "uint64" => { print_values(Vec::<u64>::load_from(input)?); Ok(()) },
                "float32" => { print_values(Vec::<f32>::load_from(input)?); Ok(()) },
                "float64" => { print_values(Vec::<f64>::load_from(input)?); Ok(()) },
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
