#[macro_export]
macro_rules! dt {
    ($t:ty) => {
        <$t as Div<Time>>::Output
    };
}

#[macro_export]
macro_rules! ddt {
    ($t:ty) => {
        dt!(<$t as Div<Time>>::Output)
    };
}

#[macro_export]
macro_rules! dddt {
    ($t:ty) => {
        ddt!(<$t as Div<Time>>::Output)
    };
}
