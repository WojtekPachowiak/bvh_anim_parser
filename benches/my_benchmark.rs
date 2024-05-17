use criterion::{black_box, criterion_group, criterion_main, Criterion};
use bvh_anim_parser::parse::{load_bvh_from_file, load_bvh_from_string};
use std::fs;

pub fn criterion_benchmark(c: &mut Criterion) {

    fn iterate_over_cmu_dataset() {
        let paths = fs::read_dir("./benches/cmu_bvhs").unwrap();
        let mut num_frames = Vec::new();
        for path in paths {
            let (bvh_metadata, bvh_data) = load_bvh_from_file(path.unwrap().path().to_str().unwrap());
            num_frames.push(bvh_metadata.num_frames);
        }
        // // print statistics like mean, median, max, min
        // num_frames.sort();
        // println!("mean: {}", num_frames.iter().sum::<usize>() as f64 / num_frames.len() as f64);
        // println!("median: {}", num_frames[num_frames.len() / 2]);
        // println!("max: {}", num_frames.iter().max().unwrap());
        // println!("min: {}", num_frames.iter().min().unwrap());
        // println!("====")
    }

    let mut group = c.benchmark_group("sample-size-example");
    group.sample_size(10);
    group.bench_function("cmu part", |b| b.iter(|| black_box(iterate_over_cmu_dataset())));
    group.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);