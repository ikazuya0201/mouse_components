macro_rules! dt {
    ($t:ty) => {
        <$t as Div<Time>>::Output
    };
}

macro_rules! ddt {
    ($t:ty) => {
        dt!(<$t as Div<Time>>::Output)
    };
}

macro_rules! dddt {
    ($t:ty) => {
        ddt!(<$t as Div<Time>>::Output)
    };
}
