#[doc(hidden)]
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
        $builder: ident:
        {
            $(#[$meta:meta])*
            pub struct $name: ident <$($tt: tt),*> {
                $($field_name: ident: $type: ty,)*
            }
        }
    ) => {
        $(#[$meta])*
        pub struct $name<$($tt),*> {
            $($field_name: $type),*
        }

        impl<$($tt),*> $name<$($tt),*> {
            pub fn builder() -> $builder<$($tt),*> {
                $builder::new()
            }
        }

        pub struct $builder<$($tt),*> {
            $($field_name: Option<$type>),*
        }

        impl<$($tt),*> $builder<$($tt),*> {
            pub fn new() -> Self {
                Self {
                    $($field_name: None),*
                }
            }

            pub fn build(&mut self) -> crate::utils::builder::BuilderResult<$name<$($tt),*>> {
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
    }
}
