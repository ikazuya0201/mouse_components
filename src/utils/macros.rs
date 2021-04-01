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

#[macro_export]
macro_rules! impl_setter {
    (
        $(#[$meta:meta])*
        $method_name: ident: $field_name: ident: $type: ty
    ) => {
        $(#[$meta])*
        pub fn $method_name(&mut self, $field_name: $type) -> &mut Self {
            self.$field_name = Some($field_name);
            self
        }
    };

    (
        $(#[$meta:meta])*
        $field_name: ident: $type: ty
    ) => {
        impl_setter!(
            $(#[$meta])*
            $field_name: $field_name: $type
        );
    };
}

#[macro_export]
macro_rules! get_or_err {
    ($self: ident . $field_name: ident) => {{
        crate::utils::builder::ok_or($self.$field_name.take(), core::stringify!($field_name))?
    }};
}
