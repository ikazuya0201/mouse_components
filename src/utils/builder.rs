#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RequiredFieldEmptyError {
    field_name: &'static str,
}

impl core::fmt::Display for RequiredFieldEmptyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(
            f,
            "Build failed: the field `{}` is required",
            self.field_name
        )
    }
}

pub fn ok_or<T>(value: Option<T>, field_name: &'static str) -> Result<T, RequiredFieldEmptyError> {
    value.ok_or(RequiredFieldEmptyError { field_name })
}

pub type BuilderResult<T> = Result<T, RequiredFieldEmptyError>;
