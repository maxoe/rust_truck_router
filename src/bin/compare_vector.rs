use stud_rust_base::{io::*, cli::CliErr};
use std::{env, fmt::Display, error::Error};

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args();
    args.next();

    match &args.collect::<Vec<String>>()[..] {
        [data_type, input1, input2] => {
            match data_type.as_ref() {
                "i8" => { compare_values(&Vec::<i8>::load_from(input1)?, &Vec::<i8>::load_from(input2)?); Ok(()) },
                "u8" => { compare_values(&Vec::<u8>::load_from(input1)?, &Vec::<u8>::load_from(input2)?); Ok(()) },
                "i16" => { compare_values(&Vec::<i16>::load_from(input1)?, &Vec::<i16>::load_from(input2)?); Ok(()) },
                "u16" => { compare_values(&Vec::<u16>::load_from(input1)?, &Vec::<u16>::load_from(input2)?); Ok(()) },
                "i32" => { compare_values(&Vec::<i32>::load_from(input1)?, &Vec::<i32>::load_from(input2)?); Ok(()) },
                "u32" => { compare_values(&Vec::<u32>::load_from(input1)?, &Vec::<u32>::load_from(input2)?); Ok(()) },
                "i64" => { compare_values(&Vec::<i64>::load_from(input1)?, &Vec::<i64>::load_from(input2)?); Ok(()) },
                "u64" => { compare_values(&Vec::<u64>::load_from(input1)?, &Vec::<u64>::load_from(input2)?); Ok(()) },
                "f32" => { compare_values(&Vec::<f32>::load_from(input1)?, &Vec::<f32>::load_from(input2)?); Ok(()) },
                "f64" => { compare_values(&Vec::<f64>::load_from(input1)?, &Vec::<f64>::load_from(input2)?); Ok(()) },
                "int8" => { compare_values(&Vec::<i8>::load_from(input1)?, &Vec::<i8>::load_from(input2)?); Ok(()) },
                "uint8" => { compare_values(&Vec::<u8>::load_from(input1)?, &Vec::<u8>::load_from(input2)?); Ok(()) },
                "int16" => { compare_values(&Vec::<i16>::load_from(input1)?, &Vec::<i16>::load_from(input2)?); Ok(()) },
                "uint16" => { compare_values(&Vec::<u16>::load_from(input1)?, &Vec::<u16>::load_from(input2)?); Ok(()) },
                "int32" => { compare_values(&Vec::<i32>::load_from(input1)?, &Vec::<i32>::load_from(input2)?); Ok(()) },
                "uint32" => { compare_values(&Vec::<u32>::load_from(input1)?, &Vec::<u32>::load_from(input2)?); Ok(()) },
                "int64" => { compare_values(&Vec::<i64>::load_from(input1)?, &Vec::<i64>::load_from(input2)?); Ok(()) },
                "uint64" => { compare_values(&Vec::<u64>::load_from(input1)?, &Vec::<u64>::load_from(input2)?); Ok(()) },
                "float32" => { compare_values(&Vec::<f32>::load_from(input1)?, &Vec::<f32>::load_from(input2)?); Ok(()) },
                "float64" => { compare_values(&Vec::<f64>::load_from(input1)?, &Vec::<f64>::load_from(input2)?); Ok(()) },
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
    eprintln!("Usage: decode_vector data_type vector1_file vector2_file

Compares two vectors of elements in binary format. data_type can be one of
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

fn compare_values<T>(values1: &[T], values2: &[T]) where
    T: Display,
    T: PartialOrd
{
    if values1.len() != values2.len() {
        println!("0");
        eprintln!("Can only compare vectors of equal size. The first vector has {} elements. The second vector has {} elements.", values1.len(), values2.len());
        return
    }

    let mut v1_smaller_count = 0;
    let mut v2_smaller_count = 0;
    let mut first_diff = None;

    for (i, (v1, v2)) in values1.iter().zip(values2.iter()).enumerate() {
        if v1 < v2 { v1_smaller_count += 1; }
        if v2 < v1 { v2_smaller_count += 1; }

        if first_diff.is_none() && v1 != v2 {
            first_diff = Some(i)
        }
    }

    match first_diff {
        Some(index) => {
            eprintln!("The vectors differ.");
            eprintln!("{} elements are smaller in the first vector.", v1_smaller_count);
            eprintln!("{} elements are smaller in the second vector.", v2_smaller_count);
            eprintln!("{} elements are the same.", values1.len() - v1_smaller_count - v2_smaller_count);
            println!("{}", values1.len() - v1_smaller_count - v2_smaller_count);
            eprintln!("{} elements are different.", v1_smaller_count + v2_smaller_count);
            eprintln!("The vectors have {} elements.", values1.len());
            eprintln!("The first element that differs is at index {}.", index);
        },
        None => {
            println!("{}", values1.len());
            eprintln!("The vectors are the same and have {} elements.", values1.len());
        },
    }
}
