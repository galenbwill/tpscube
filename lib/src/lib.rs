mod action;
mod analysis;
mod common;
mod cube2x2x2;
mod cube3x3x3;
mod cube4x4x4;
mod rand;
mod request;
mod tables;

#[cfg(feature = "storage")]
mod future;
#[cfg(feature = "storage")]
mod history;
#[cfg(feature = "storage")]
mod import;
#[cfg(feature = "storage")]
mod storage;
#[cfg(feature = "storage")]
mod sync;

#[cfg(feature = "bluetooth")]
mod bluetooth;

#[allow(dead_code, unused_imports)]
mod action_generated;
#[allow(dead_code, unused_imports)]
mod index_generated;

pub use crate::rand::{RandomSource, SimpleSeededRandomSource, StandardRandomSource};
pub use action::{Action, StoredAction};
pub use analysis::{
    Analysis, AnalysisStepSummary, AnalysisSubstepTime, AnalysisSummary, CFOPAnalysis,
    CFOPPartialAnalysis, CFOPProgress, CrossAnalysis, CubeWithSolution, F2LPairAnalysis,
    FinalAlignmentAnalysis, OLLAlgorithm, OLLAnalysis, PLLAlgorithm, PLLAnalysis, PartialAnalysis,
    PartialAnalysisMethod, SolveAnalysis,
};
pub use common::{
    parse_move_string, parse_timed_move_string, Average, BestSolve, Color, Corner, CornerPiece,
    Cube, CubeFace, FaceRotation, InitialCubeState, ListAverage, Move, MoveSequence, Penalty,
    RotationDirection, Solve, SolveList, SolveType, TimedMove,
};
pub use cube2x2x2::{Cube2x2x2, Cube2x2x2Faces};
pub use cube3x3x3::{Cube3x3x3, Cube3x3x3Faces, Edge3x3x3, EdgePiece3x3x3};
pub use cube4x4x4::{Cube4x4x4, Cube4x4x4Faces, Edge4x4x4, EdgePiece4x4x4};
pub use request::{SyncRequest, SyncResponse, SYNC_API_VERSION};

#[cfg(feature = "storage")]
pub use history::{History, HistoryLoadProgress, Session};
#[cfg(feature = "storage")]
pub use sync::SyncStatus;

#[cfg(feature = "bluetooth")]
pub use bluetooth::{
    AvailableDevice, BluetoothCube, BluetoothCubeEvent, BluetoothCubeState, BluetoothCubeType,
    MoveListenerHandle,
};

#[cfg(not(feature = "no_solver"))]
pub use cube2x2x2::scramble_2x2x2;
#[cfg(not(feature = "no_solver"))]
pub use cube3x3x3::{scramble_3x3x3, scramble_3x3x3_fast};
#[cfg(not(feature = "no_solver"))]
pub use cube4x4x4::{scramble_4x4x4, scramble_4x4x4_fast};

#[cfg(test)]
mod tests {
    use crate::{
        Cube, Cube2x2x2, Cube2x2x2Faces, Cube3x3x3, Cube3x3x3Faces, Cube4x4x4, Cube4x4x4Faces,
        InitialCubeState, Move, MoveSequence, SimpleSeededRandomSource,
    };
    use std::convert::TryFrom;

    fn basic_small_cube_movement<T: Cube + InitialCubeState + std::fmt::Display>() {
        let mut cube = T::new();
        assert!(cube.is_solved(), "initial state is not solved\n{}", cube);
        cube.do_move(Move::U);
        assert!(!cube.is_solved(), "not unsolved after U\n{}", cube);
        cube.do_move(Move::Up);
        assert!(cube.is_solved(), "not solved after U U'\n{}", cube);

        let y_perm: &'static [Move] = &[
            Move::F,
            Move::R,
            Move::Up,
            Move::Rp,
            Move::Up,
            Move::R,
            Move::U,
            Move::Rp,
            Move::Fp,
            Move::R,
            Move::U,
            Move::Rp,
            Move::Up,
            Move::Rp,
            Move::F,
            Move::R,
            Move::Fp,
        ];
        for _ in 0..2 {
            cube.do_moves(y_perm);
        }
        assert!(cube.is_solved(), "not solved after 2x Y perm\n{}", cube);

        let scramble: &'static [Move] = &[
            Move::D2,
            Move::R2,
            Move::B2,
            Move::L,
            Move::U2,
            Move::R,
            Move::D2,
            Move::Lp,
            Move::B2,
            Move::R2,
            Move::D2,
            Move::Fp,
            Move::Lp,
            Move::D,
            Move::L,
            Move::R2,
            Move::D,
            Move::B,
            Move::U,
            Move::L2,
        ];
        let inv_scramble = scramble.inverse();
        cube.do_moves(scramble);
        cube.do_moves(&inv_scramble);
        assert!(
            cube.is_solved(),
            "not solved after static scramble and inverse\n{}",
            cube
        );
    }

    fn basic_4x4x4_cube_movement<T: Cube + InitialCubeState + std::fmt::Display>() {
        let mut cube = T::new();
        assert!(cube.is_solved(), "initial state is not solved\n{}", cube);
        cube.do_move(Move::U);
        assert!(!cube.is_solved(), "not unsolved after U\n{}", cube);
        cube.do_move(Move::Up);
        assert!(cube.is_solved(), "not solved after U U'\n{}", cube);

        let y_perm: &'static [Move] = &[
            Move::F,
            Move::R,
            Move::Up,
            Move::Rp,
            Move::Up,
            Move::R,
            Move::U,
            Move::Rp,
            Move::Fp,
            Move::R,
            Move::U,
            Move::Rp,
            Move::Up,
            Move::Rp,
            Move::F,
            Move::R,
            Move::Fp,
        ];
        for _ in 0..2 {
            cube.do_moves(y_perm);
        }
        assert!(cube.is_solved(), "not solved after 2x Y perm\n{}", cube);

        let oll_parity: &'static [Move] = &[
            Move::Rwp,
            Move::U2,
            Move::Lw,
            Move::F2,
            Move::Lwp,
            Move::F2,
            Move::Rw2,
            Move::U2,
            Move::Rw,
            Move::U2,
            Move::Rwp,
            Move::U2,
            Move::F2,
            Move::Rw2,
            Move::F2,
        ];
        for _ in 0..2 {
            cube.do_moves(oll_parity);
        }
        assert!(cube.is_solved(), "not solved after 2x OLL parity\n{}", cube);

        let scramble: &'static [Move] = &[
            Move::L,
            Move::B,
            Move::U,
            Move::B2,
            Move::R2,
            Move::U2,
            Move::R2,
            Move::D2,
            Move::Up,
            Move::B2,
            Move::U2,
            Move::R2,
            Move::F,
            Move::D2,
            Move::Lp,
            Move::R,
            Move::Fp,
            Move::R,
            Move::Up,
            Move::Rw2,
            Move::Bp,
            Move::L2,
            Move::Fw2,
            Move::Uw2,
            Move::Rw2,
            Move::Dp,
            Move::Rw2,
            Move::Up,
            Move::B,
            Move::Dp,
            Move::B2,
            Move::U2,
            Move::Rw,
            Move::Bp,
            Move::R,
            Move::Fp,
            Move::Rw,
            Move::Fw,
            Move::Uw,
            Move::F2,
            Move::Uwp,
            Move::U,
            Move::Dp,
            Move::Rw,
        ];
        let inv_scramble = scramble.inverse();
        cube.do_moves(scramble);
        cube.do_moves(&inv_scramble);
        assert!(
            cube.is_solved(),
            "not solved after static scramble and inverse\n{}",
            cube
        );
    }

    #[test]
    fn basic_2x2x2_face_movement() {
        basic_small_cube_movement::<Cube2x2x2Faces>();
    }

    #[test]
    fn basic_2x2x2_piece_movement() {
        basic_small_cube_movement::<Cube2x2x2>();
    }

    #[test]
    fn basic_3x3x3_face_movement() {
        basic_small_cube_movement::<Cube3x3x3Faces>();
    }

    #[test]
    fn basic_3x3x3_piece_movement() {
        basic_small_cube_movement::<Cube3x3x3>();
    }

    #[test]
    fn basic_4x4x4_face_movement() {
        basic_4x4x4_cube_movement::<Cube4x4x4Faces>();
    }

    #[test]
    fn basic_4x4x4_piece_movement() {
        basic_4x4x4_cube_movement::<Cube4x4x4>();
    }

    #[test]
    fn oll_parity_4x4x4() {
        let mut cube = Cube4x4x4::new();
        assert!(!cube.oll_parity(), "oll parity on initial state");

        // Performing any 3x3x3 moves should not impact OLL parity
        let mut rng = SimpleSeededRandomSource::new();
        for _ in 0..100 {
            let mv = Move::sourced_random_3x3x3(&mut rng);
            cube.do_move(mv);
            assert!(!cube.oll_parity(), "oll parity after 3x3x3 moves");
        }

        // Perform OLL parity algorithm and ensure OLL parity state
        let oll_parity: &'static [Move] = &[
            Move::Rwp,
            Move::U2,
            Move::Lw,
            Move::F2,
            Move::Lwp,
            Move::F2,
            Move::Rw2,
            Move::U2,
            Move::Rw,
            Move::U2,
            Move::Rwp,
            Move::U2,
            Move::F2,
            Move::Rw2,
            Move::F2,
        ];
        cube.do_moves(oll_parity);
        assert!(cube.oll_parity(), "no oll parity after parity algorithm");

        // Perform more 3x3x3 moves and ensure OLL parity state persists
        for _ in 0..100 {
            let mv = Move::sourced_random_3x3x3(&mut rng);
            cube.do_move(mv);
            assert!(cube.oll_parity(), "lost oll parity after 3x3x3 moves");
        }

        // Perform OLL parity algorithm again and ensure that OLL parity state
        // is no longer the case
        cube.do_moves(oll_parity);
        assert!(!cube.oll_parity(), "oll parity after 2x parity algorithm");
    }

    #[test]
    fn matching_2x2x2_formats() {
        for mv in &[Move::U, Move::L, Move::R, Move::D, Move::F, Move::B] {
            let mut pieces = Cube2x2x2::new();
            let mut faces = Cube2x2x2Faces::new();
            pieces.do_move(*mv);
            faces.do_move(*mv);
            let pieces_conv = faces.as_pieces();
            let faces_conv = pieces.as_faces();
            assert_eq!(
                pieces, pieces_conv,
                "face format incorrectly converted to piece format\n\
                Face format:\n{}\
                Piece format:\n{}",
                faces, pieces_conv
            );
            assert_eq!(
                faces, faces_conv,
                "piece format incorrectly converted to face format\n\
                Piece format:\n{}\
                Face format:\n{}",
                pieces, faces_conv
            );
        }

        let mut pieces = Cube2x2x2::new();
        let mut faces = Cube2x2x2Faces::new();
        let mut rng = SimpleSeededRandomSource::new();
        for _ in 0..100 {
            let mv = Move::sourced_random_2x2x2(&mut rng);
            pieces.do_move(mv);
            faces.do_move(mv);
        }

        let pieces_conv = faces.as_pieces();
        let faces_conv = pieces.as_faces();
        assert_eq!(
            pieces, pieces_conv,
            "face format incorrectly converted to piece format\n\
            Face format:\n{}\
            Piece format:\n{}",
            faces, pieces_conv
        );
        assert_eq!(
            faces, faces_conv,
            "piece format incorrectly converted to face format\n\
            Piece format:\n{}\
            Face format:\n{}",
            pieces, faces_conv
        );
    }

    #[test]
    fn matching_3x3x3_formats() {
        for mv in 0..Move::count_3x3x3() {
            let mv = Move::try_from(mv as u8).unwrap();
            let mut pieces = Cube3x3x3::new();
            let mut faces = Cube3x3x3Faces::new();
            pieces.do_move(mv);
            faces.do_move(mv);
            let pieces_conv = faces.as_pieces();
            let faces_conv = pieces.as_faces();
            assert_eq!(
                pieces, pieces_conv,
                "face format incorrectly converted to piece format\n\
                Face format:\n{}\
                Piece format:\n{}",
                faces, pieces_conv
            );
            assert_eq!(
                faces, faces_conv,
                "piece format incorrectly converted to face format\n\
                Piece format:\n{}\
                Face format:\n{}",
                pieces, faces_conv
            );
        }

        let mut pieces = Cube3x3x3::new();
        let mut faces = Cube3x3x3Faces::new();
        let mut rng = SimpleSeededRandomSource::new();
        for _ in 0..100 {
            let mv = Move::sourced_random_3x3x3(&mut rng);
            pieces.do_move(mv);
            faces.do_move(mv);
        }

        let pieces_conv = faces.as_pieces();
        let faces_conv = pieces.as_faces();
        assert_eq!(
            pieces, pieces_conv,
            "face format incorrectly converted to piece format\n\
            Face format:\n{}\
            Piece format:\n{}",
            faces, pieces_conv
        );
        assert_eq!(
            faces, faces_conv,
            "piece format incorrectly converted to face format\n\
            Piece format:\n{}\
            Face format:\n{}",
            pieces, faces_conv
        );
    }

    #[test]
    fn matching_4x4x4_formats() {
        for mv in 0..Move::count_4x4x4() {
            let mv = Move::try_from(mv as u8).unwrap();
            let mut pieces = Cube4x4x4::new();
            let mut faces = Cube4x4x4Faces::new();
            pieces.do_move(mv);
            faces.do_move(mv);
            let pieces_conv = faces.as_pieces();
            let faces_conv = pieces.as_faces();
            assert_eq!(
                pieces, pieces_conv,
                "face format incorrectly converted to piece format\n\
                Face format:\n{}\
                Piece format:\n{}",
                faces, pieces_conv
            );
            assert_eq!(
                faces, faces_conv,
                "piece format incorrectly converted to face format\n\
                Piece format:\n{}\
                Face format:\n{}",
                pieces, faces_conv
            );
        }

        let mut pieces = Cube4x4x4::new();
        let mut faces = Cube4x4x4Faces::new();
        let mut rng = SimpleSeededRandomSource::new();
        for _ in 0..100 {
            let mv = Move::sourced_random_4x4x4(&mut rng);
            pieces.do_move(mv);
            faces.do_move(mv);
        }

        let pieces_conv = faces.as_pieces();
        let faces_conv = pieces.as_faces();
        assert_eq!(
            pieces, pieces_conv,
            "face format incorrectly converted to piece format\n\
            Face format:\n{}\
            Piece format:\n{}",
            faces, pieces_conv
        );
        assert_eq!(
            faces, faces_conv,
            "piece format incorrectly converted to face format\n\
            Piece format:\n{}\
            Face format:\n{}",
            pieces, faces_conv
        );
    }

    #[test]
    fn solve_2x2x2() {
        let mut rng = SimpleSeededRandomSource::new();
        for _ in 0..10 {
            let mut cube = Cube2x2x2::sourced_random(&mut rng);
            let solution = cube.solve().unwrap();
            let initial = cube.clone();
            for mv in &solution {
                cube.do_move(*mv);
            }
            assert!(
                cube.is_solved(),
                "cube solution invalid\n\
                Initial state:\n{}\
                Solution:\n{:?}\
                Final state:\n{}",
                initial,
                solution,
                cube
            );
        }

        for _ in 0..10 {
            let mut cube = Cube3x3x3::sourced_random(&mut rng);
            let solution = cube.solve_fast().unwrap();
            let initial = cube.clone();
            cube.do_moves(&solution);
            assert!(
                cube.is_solved(),
                "cube solution invalid\n\
                Initial state:\n{}\
                Solution:\n{:?}\
                Final state:\n{}",
                initial,
                solution,
                cube
            );
        }
    }

    #[test]
    fn solve_3x3x3() {
        let mut rng = SimpleSeededRandomSource::new();
        for _ in 0..10 {
            let mut cube = Cube3x3x3::sourced_random(&mut rng);
            let solution = cube.solve().unwrap();
            let initial = cube.clone();
            for mv in &solution {
                cube.do_move(*mv);
            }
            assert!(
                cube.is_solved(),
                "cube solution invalid\n\
                Initial state:\n{}\
                Solution:\n{:?}\
                Final state:\n{}",
                initial,
                solution,
                cube
            );
        }

        for _ in 0..10 {
            let mut cube = Cube3x3x3::sourced_random(&mut rng);
            let solution = cube.solve_fast().unwrap();
            let initial = cube.clone();
            cube.do_moves(&solution);
            assert!(
                cube.is_solved(),
                "cube solution invalid\n\
                Initial state:\n{}\
                Solution:\n{:?}\
                Final state:\n{}",
                initial,
                solution,
                cube
            );
        }
    }
}
