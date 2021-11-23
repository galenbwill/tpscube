#[macro_export]
macro_rules! function {
    () => {{
        fn f() {}
        fn type_name_of<T>(_: T) -> &'static str {
            std::any::type_name::<T>()
        }
        let name = type_name_of(f);
        &name[..name.len() - 3]

        // Find and cut the rest of the path
        // match &name[..name.len() - 3].rfind(':') {
        //     Some(pos) => &name[pos + 1..name.len() - 3],
        //     None => &name[..name.len() - 3],
        // }
        // match &name[..name.len() - 3].rfind(':') {
        //     Some(pos) => match &name[..pos + 1].rfind(':') {
        //         Some(pos2) => &name[pos2 + 1..name.len() - 3],
        //         None => &name[pos + 1..name.len() - 3],
        //     },
        //     None => &name[..name.len() - 3],
        // }
    }};
}

// pub use function;
