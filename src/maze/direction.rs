#[derive(Clone, Copy, PartialEq, Eq, core::fmt::Debug)]
pub enum AbsoluteDirection {
    North,
    NorthEast,
    East,
    SouthEast,
    South,
    SouthWest,
    West,
    NorthWest,
}

#[derive(Clone, Copy, PartialEq, Eq, core::fmt::Debug)]
pub enum RelativeDirection {
    Front,
    FrontRight,
    Right,
    BackRight,
    Back,
    BackLeft,
    Left,
    FrontLeft,
}

impl AbsoluteDirection {
    pub fn rotate(&self, direction: RelativeDirection) -> Self {
        use AbsoluteDirection::*;
        use RelativeDirection::*;
        match direction {
            Front => match self {
                North => North,
                NorthEast => NorthEast,
                East => East,
                SouthEast => SouthEast,
                South => South,
                SouthWest => SouthWest,
                West => West,
                NorthWest => NorthWest,
            },
            FrontRight => match self {
                North => NorthEast,
                NorthEast => East,
                East => SouthEast,
                SouthEast => South,
                South => SouthWest,
                SouthWest => West,
                West => NorthWest,
                NorthWest => North,
            },
            Right => match self {
                North => East,
                NorthEast => SouthEast,
                East => SouthEast,
                SouthEast => SouthWest,
                South => West,
                SouthWest => NorthWest,
                West => North,
                NorthWest => NorthEast,
            },
            BackRight => match self {
                North => SouthEast,
                NorthEast => South,
                East => SouthWest,
                SouthEast => West,
                South => NorthWest,
                SouthWest => North,
                West => NorthEast,
                NorthWest => East,
            },
            Back => match self {
                North => South,
                NorthEast => SouthWest,
                East => West,
                SouthEast => NorthWest,
                South => North,
                SouthWest => NorthEast,
                West => East,
                NorthWest => SouthEast,
            },
            BackLeft => match self {
                North => SouthWest,
                NorthEast => West,
                East => NorthWest,
                SouthEast => North,
                South => NorthEast,
                SouthWest => East,
                West => SouthEast,
                NorthWest => South,
            },
            Left => match self {
                North => West,
                NorthEast => NorthWest,
                East => North,
                SouthEast => NorthEast,
                South => East,
                SouthWest => SouthEast,
                West => South,
                NorthWest => SouthWest,
            },
            FrontLeft => match self {
                North => NorthWest,
                NorthEast => North,
                East => NorthEast,
                SouthEast => East,
                South => SouthEast,
                SouthWest => South,
                West => SouthWest,
                NorthWest => West,
            },
        }
    }

    pub fn relative(&self, to: AbsoluteDirection) -> RelativeDirection {
        RelativeDirection::relative(*self, to)
    }
}

impl RelativeDirection {
    pub fn relative(from: AbsoluteDirection, to: AbsoluteDirection) -> Self {
        use AbsoluteDirection::*;
        use RelativeDirection::*;
        match from {
            North => match to {
                North => Front,
                NorthEast => FrontRight,
                East => Right,
                SouthEast => BackRight,
                South => Back,
                SouthWest => BackLeft,
                West => Left,
                NorthWest => FrontLeft,
            },
            NorthEast => match to {
                North => FrontLeft,
                NorthEast => Front,
                East => FrontRight,
                SouthEast => Right,
                South => BackRight,
                SouthWest => Back,
                West => BackLeft,
                NorthWest => Left,
            },
            East => match to {
                North => Left,
                NorthEast => FrontLeft,
                East => Front,
                SouthEast => FrontRight,
                South => Right,
                SouthWest => BackRight,
                West => Back,
                NorthWest => BackLeft,
            },
            SouthEast => match to {
                North => BackLeft,
                NorthEast => Left,
                East => FrontLeft,
                SouthEast => Front,
                South => FrontRight,
                SouthWest => Right,
                West => BackRight,
                NorthWest => Back,
            },
            South => match to {
                North => Back,
                NorthEast => BackLeft,
                East => Left,
                SouthEast => FrontLeft,
                South => Front,
                SouthWest => FrontRight,
                West => Right,
                NorthWest => BackRight,
            },
            SouthWest => match to {
                North => BackRight,
                NorthEast => Back,
                East => BackLeft,
                SouthEast => Left,
                South => FrontLeft,
                SouthWest => Front,
                West => FrontRight,
                NorthWest => Right,
            },
            West => match to {
                North => Right,
                NorthEast => BackRight,
                East => Back,
                SouthEast => BackLeft,
                South => Left,
                SouthWest => FrontLeft,
                West => Front,
                NorthWest => FrontRight,
            },
            NorthWest => match to {
                North => FrontRight,
                NorthEast => Right,
                East => BackRight,
                SouthEast => Back,
                South => BackLeft,
                SouthWest => Left,
                West => FrontLeft,
                NorthWest => Front,
            },
        }
    }
}
