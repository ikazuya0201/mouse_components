#[macro_export]
macro_rules! impl_with_getter {
    (
        $(#[$meta:meta])*
        pub struct $name: ident <$($tt: tt),*> {
            $($field_name: ident: $type: ty,)*
        }
    ) => {
        $(#[$meta])*
        pub struct $name<$($tt),*> {
            $($field_name: $type),*
        }

        impl<$($tt),*> $name<$($tt),*> {
            $(
                pub fn $field_name(&self) -> &$type {
                    &self.$field_name
                }
            )*
        }
    };
}
