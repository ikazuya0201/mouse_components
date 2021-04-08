#[doc(hidden)]
#[macro_export]
macro_rules! impl_with_getter {
    (
        $(#[$meta:meta])*
        pub struct $name: ident $(<$(const $const: ident : usize),*>)? {
            $(
                $(#[$field_meta:meta])*
                $field_name: ident: $type: ty,
            )*
        }
    ) => {
        impl_with_getter! {
            $(#[$meta])*
            pub struct $name$(<; $(const $const: usize),*>)? {
                $(
                    $(#[$field_meta])*
                    $field_name: $type,
                )*
            }
        }
    };

    (
        $(#[$meta:meta])*
        pub struct $name: ident $(<$($tt: ident),*>)? {
            $(
                $(#[$field_meta:meta])*
                $field_name: ident: $type: ty,
            )*
        }
    ) => {
        impl_with_getter! {
            $(#[$meta])*
            pub struct $name$(<$($tt),*;>)? {
                $(
                    $(#[$field_meta])*
                    $field_name: $type,
                )*
            }
        }
    };

    (
        $(#[$meta:meta])*
        pub struct $name: ident <$($tt: ident),*; $(const $const: ident : usize),*> {
            $(
                $(#[$field_meta:meta])*
                $field_name: ident: $type: ty,
            )*
        }
    ) => {
        $(#[$meta])*
        pub struct $name<$($tt,)* $(const $const: usize),*> {
            $(
                $(#[$field_meta])*
                $field_name: $type
            ),*
        }

        impl<$($tt,)* $(const $const: usize),*> $name<$($tt,)* $($const),*> {
            $(
                pub fn $field_name(&self) -> &$type {
                    &self.$field_name
                }
            )*
        }
    }
}

#[doc(hidden)]
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

#[doc(hidden)]
#[macro_export]
macro_rules! get_or_err {
    ($self: ident . $field_name: ident) => {{
        crate::utils::builder::ok_or($self.$field_name.take(), core::stringify!($field_name))?
    }};
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_with_builder {
    (
        $(#[$builder_meta:meta])*
        $builder: ident:
        {
            $(#[$meta:meta])*
            pub struct $name: ident <$($tt: tt),*> {
                $($field_name: ident: $type: ty,)*
            }
        }
    ) => {
        impl_with_builder! {
            $(#[$builder_meta])*
            $builder:
            {
                $(#[$meta])*
                pub struct $name<$($tt),*;> {
                    $($field_name: $type,)*
                }
            }
        }
    };

    (
        $(#[$builder_meta:meta])*
        $builder: ident:
        {
            $(#[$meta:meta])*
            pub struct $name: ident <$($tt: tt),*; $(const $const: ident : usize),*> {
                $($field_name: ident: $type: ty,)*
            }
        }
    ) => {
        $(#[$meta])*
        pub struct $name<$($tt),*, $(const $const: usize),*> {
            $($field_name: $type),*
        }

        impl<$($tt),*, $(const $const: usize),*> $name<$($tt),*, $($const),*> {
            pub fn builder() -> $builder<$($tt),*, $($const),*> {
                $builder::new()
            }
        }

        $(#[$builder_meta])*
        pub struct $builder<$($tt),*, $(const $const: usize),*> {
            $($field_name: Option<$type>),*
        }

        impl<$($tt),*, $(const $const: usize),*> $builder<$($tt),*, $($const),*> {
            pub fn new() -> Self {
                Self {
                    $($field_name: None),*
                }
            }

            pub fn build(&mut self) -> crate::utils::builder::BuilderResult<$name<$($tt),*, $($const),*>> {
                Ok(
                    $name {
                        $($field_name: get_or_err!(self.$field_name),)*
                    }
                )
            }

            $(
                pub fn $field_name(&mut self, $field_name: $type) -> &mut Self {
                    self.$field_name = Some($field_name);
                    self
                }
            )*
        }
    };
}
