use crate::{
    Color, Corner, CornerPiece, Cube, CubeFace, FaceRotation, InitialCubeState, Move, RandomSource,
    RotationDirection,
};
use num_enum::TryFromPrimitive;
use std::collections::BTreeMap;
use std::convert::TryFrom;

#[cfg(not(feature = "no_solver"))]
use crate::common::{CornerOrientationMoveTable, CornerPermutationMoveTable, MoveSequence};
#[cfg(not(feature = "no_solver"))]
use std::convert::TryInto;

#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy, TryFromPrimitive)]
/// Identification of an edge piece. Names come from the faces of the cube this edge
/// belongs to on a solved cube.
pub enum Edge3x3x3 {
    UR = 0,
    UF = 1,
    UL = 2,
    UB = 3,
    DR = 4,
    DF = 5,
    DL = 6,
    DB = 7,
    FR = 8,
    FL = 9,
    BL = 10,
    BR = 11,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct EdgePiece3x3x3 {
    pub piece: Edge3x3x3,
    pub orientation: u8,
}

#[derive(Debug, PartialEq, Eq, Clone)]
/// A 3x3x3 cube represented in piece format (optimal for computational algorithms).
pub struct Cube3x3x3 {
    corners: [CornerPiece; 8],
    edges: [EdgePiece3x3x3; 12],
}

#[derive(Debug, PartialEq, Eq, Clone)]
/// A 3x3x3 cube represented in face color format (easy to use, matches visuals).
pub struct Cube3x3x3Faces {
    state: [Color; 6 * 9],
}

pub(crate) enum FaceRowOrColumn {
    RowLeftToRight(CubeFace, usize),
    RowRightToLeft(CubeFace, usize),
    ColumnTopDown(CubeFace, usize),
    ColumnBottomUp(CubeFace, usize),
}

impl FaceRowOrColumn {
    pub fn idx(&self, idx: usize) -> usize {
        match self {
            FaceRowOrColumn::RowLeftToRight(face, row) => Cube3x3x3Faces::idx(*face, *row, idx),
            FaceRowOrColumn::RowRightToLeft(face, row) => Cube3x3x3Faces::idx(*face, *row, 2 - idx),
            FaceRowOrColumn::ColumnTopDown(face, col) => Cube3x3x3Faces::idx(*face, idx, *col),
            FaceRowOrColumn::ColumnBottomUp(face, col) => Cube3x3x3Faces::idx(*face, 2 - idx, *col),
        }
    }
}

#[cfg(not(feature = "no_solver"))]
struct EdgeOrientationMoveTable;
#[cfg(not(feature = "no_solver"))]
struct EquatorialEdgeSliceMoveTable;
#[cfg(not(feature = "no_solver"))]
struct Phase2EdgePermutationMoveTable;
#[cfg(not(feature = "no_solver"))]
struct Phase2EquatorialEdgePermutationMoveTable;
#[cfg(not(feature = "no_solver"))]
struct CornerOrientationEdgeSlicePruneTable;
#[cfg(not(feature = "no_solver"))]
struct EdgeOrientationPruneTable;
#[cfg(not(feature = "no_solver"))]
struct CombinedOrientationPruneTable;
#[cfg(not(feature = "no_solver"))]
struct CornerEdgePermutationPruneTable;
#[cfg(not(feature = "no_solver"))]
struct Phase1CornerPermutationPruneTable;
#[cfg(not(feature = "no_solver"))]
struct Phase2EdgePermutationPruneTable;

#[cfg(not(feature = "no_solver"))]
impl EdgeOrientationMoveTable {
    fn get(idx: u16, mv: Move) -> u16 {
        let offset = idx as usize * Move::count_3x3x3() * 2 + mv as u8 as usize * 2;
        u16::from_le_bytes(
            crate::tables::solve::CUBE3_EDGE_ORIENTATION_MOVE_TABLE[offset..offset + 2]
                .try_into()
                .unwrap(),
        )
    }
}

#[cfg(not(feature = "no_solver"))]
impl EquatorialEdgeSliceMoveTable {
    fn get(idx: u16, mv: Move) -> u16 {
        let offset = idx as usize * Move::count_3x3x3() * 2 + mv as u8 as usize * 2;
        u16::from_le_bytes(
            crate::tables::solve::CUBE3_EQUATORIAL_EDGE_SLICE_MOVE_TABLE[offset..offset + 2]
                .try_into()
                .unwrap(),
        )
    }
}

#[cfg(not(feature = "no_solver"))]
impl Phase2EdgePermutationMoveTable {
    fn get(idx: u16, mv: Move) -> u16 {
        let offset = idx as usize * Move::count_3x3x3() * 2 + mv as u8 as usize * 2;
        u16::from_le_bytes(
            crate::tables::solve::CUBE3_PHASE_2_EDGE_PERMUTATION_MOVE_TABLE[offset..offset + 2]
                .try_into()
                .unwrap(),
        )
    }
}

#[cfg(not(feature = "no_solver"))]
impl Phase2EquatorialEdgePermutationMoveTable {
    fn get(idx: u16, mv: Move) -> u16 {
        let offset = idx as usize * Move::count_3x3x3() * 2 + mv as u8 as usize * 2;
        u16::from_le_bytes(
            crate::tables::solve::CUBE3_PHASE_2_EQUATORIAL_EDGE_PERMUTATION_MOVE_TABLE
                [offset..offset + 2]
                .try_into()
                .unwrap(),
        )
    }
}

#[cfg(not(feature = "no_solver"))]
impl CornerOrientationEdgeSlicePruneTable {
    fn get(corner_orientation_idx: u16, edge_slice_idx: u16) -> usize {
        crate::tables::solve::CUBE3_CORNER_ORIENTATION_EDGE_SLICE_PRUNE_TABLE[corner_orientation_idx
            as usize
            * Cube3x3x3::EDGE_SLICE_INDEX_COUNT
            + edge_slice_idx as usize] as usize
    }
}

#[cfg(not(feature = "no_solver"))]
impl EdgeOrientationPruneTable {
    fn get(edge_orientation_idx: u16, edge_slice_idx: u16) -> usize {
        crate::tables::solve::CUBE3_EDGE_ORIENTATION_PRUNE_TABLE[edge_orientation_idx as usize
            * Cube3x3x3::EDGE_SLICE_INDEX_COUNT
            + edge_slice_idx as usize] as usize
    }
}

#[cfg(not(feature = "no_solver"))]
impl CombinedOrientationPruneTable {
    fn get(corner_orientation_idx: u16, edge_orientation_idx: u16) -> usize {
        crate::tables::solve::CUBE3_COMBINED_ORIENTATION_PRUNE_TABLE[corner_orientation_idx
            as usize
            * Cube3x3x3::EDGE_ORIENTATION_INDEX_COUNT
            + edge_orientation_idx as usize] as usize
    }
}

#[cfg(not(feature = "no_solver"))]
impl CornerEdgePermutationPruneTable {
    fn get(corner_permutation_idx: u16, equatorial_edge_permutation_idx: u16) -> usize {
        crate::tables::solve::CUBE3_CORNER_EDGE_PERMUTATION_PRUNE_TABLE[corner_permutation_idx
            as usize
            * Cube3x3x3::PHASE_2_EQUATORIAL_EDGE_PERMUTATION_INDEX_COUNT
            + equatorial_edge_permutation_idx as usize] as usize
    }
}

#[cfg(not(feature = "no_solver"))]
impl Phase1CornerPermutationPruneTable {
    fn get(corner_permutation_idx: u16) -> usize {
        crate::tables::solve::CUBE3_PHASE_1_CORNER_PERMUTATION_PRUNE_TABLE
            [corner_permutation_idx as usize] as usize
    }
}

#[cfg(not(feature = "no_solver"))]
impl Phase2EdgePermutationPruneTable {
    fn get(edge_permutation_idx: u16, equatorial_edge_permutation_idx: u16) -> usize {
        crate::tables::solve::CUBE3_PHASE_2_EDGE_PERMUTATION_PRUNE_TABLE[edge_permutation_idx
            as usize
            * Cube3x3x3::PHASE_2_EQUATORIAL_EDGE_PERMUTATION_INDEX_COUNT
            + equatorial_edge_permutation_idx as usize] as usize
    }
}

const fn n_choose_k(n: usize, k: usize) -> usize {
    if n < k {
        return 0;
    }
    let k = if k > n / 2 { n - k } else { k };

    let mut result = 1;
    let mut denom = 1;
    let mut i = n;
    while i > n - k {
        result *= i;
        result /= denom;
        denom += 1;
        i -= 1;
    }
    result
}

#[cfg(not(feature = "no_solver"))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct Phase1IndexCube {
    corner_orientation: u16,
    corner_permutation: u16,
    edge_orientation: u16,
    equatorial_edge_slice: u16,
}

#[cfg(not(feature = "no_solver"))]
impl Phase1IndexCube {
    fn new(pieces: &Cube3x3x3) -> Self {
        Self {
            corner_orientation: pieces.corner_orientation_index(),
            corner_permutation: pieces.corner_permutation_index(),
            edge_orientation: pieces.edge_orientation_index(),
            equatorial_edge_slice: pieces.equatorial_edge_slice_index(),
        }
    }

    fn do_move(&self, mv: Move) -> Self {
        Self {
            corner_orientation: CornerOrientationMoveTable::get(self.corner_orientation, mv),
            corner_permutation: CornerPermutationMoveTable::get(self.corner_permutation, mv),
            edge_orientation: EdgeOrientationMoveTable::get(self.edge_orientation, mv),
            equatorial_edge_slice: EquatorialEdgeSliceMoveTable::get(
                self.equatorial_edge_slice,
                mv,
            ),
        }
    }

    fn is_phase_solved(&self) -> bool {
        self.corner_orientation == 0
            && self.edge_orientation == 0
            && self.equatorial_edge_slice == 0
    }
}

#[cfg(not(feature = "no_solver"))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct Phase2IndexCube {
    corner_permutation: u16,
    edge_permutation: u16,
    equatorial_edge_permutation: u16,
}

#[cfg(not(feature = "no_solver"))]
impl Phase2IndexCube {
    fn new(phase_1: &Phase1IndexCube, pieces: &Cube3x3x3) -> Self {
        Self {
            corner_permutation: phase_1.corner_permutation,
            edge_permutation: pieces.phase_2_edge_permutation_index(),
            equatorial_edge_permutation: pieces.phase_2_equatorial_edge_permutation_index(),
        }
    }

    fn do_move(&self, mv: Move) -> Self {
        Self {
            corner_permutation: CornerPermutationMoveTable::get(self.corner_permutation, mv),
            edge_permutation: Phase2EdgePermutationMoveTable::get(self.edge_permutation, mv),
            equatorial_edge_permutation: Phase2EquatorialEdgePermutationMoveTable::get(
                self.equatorial_edge_permutation,
                mv,
            ),
        }
    }

    fn is_phase_solved(&self) -> bool {
        self.corner_permutation == 0
            && self.edge_permutation == 0
            && self.equatorial_edge_permutation == 0
    }
}

#[cfg(not(feature = "no_solver"))]
struct Solver {
    initial_state: Cube3x3x3,
    moves: Vec<Move>,
    optimal: bool,
    max_moves: usize,
    best_solution: Option<Vec<Move>>,
}

#[cfg(not(feature = "no_solver"))]
impl Solver {
    fn new(cube: &Cube3x3x3, optimal: bool) -> Self {
        Self {
            initial_state: cube.clone(),
            moves: Vec::new(),
            optimal,
            max_moves: Cube3x3x3::MAX_SOLUTION_MOVES,
            best_solution: None,
        }
    }

    fn search_phase_1(&mut self, cube: Phase1IndexCube, depth: usize) {
        // Need to go deeper. Iterate through the possible moves.
        let possible_moves = if depth == 1 {
            if self.moves.len() == 0 {
                crate::tables::solve::CUBE3_POSSIBLE_PHASE_1_LAST_MOVES
            } else {
                crate::tables::solve::CUBE3_POSSIBLE_PHASE_1_LAST_FOLLOWUP_MOVES
                    [*self.moves.last().unwrap() as u8 as usize]
            }
        } else {
            if self.moves.len() == 0 {
                crate::tables::solve::CUBE3_POSSIBLE_PHASE_1_MOVES
            } else {
                crate::tables::solve::CUBE3_POSSIBLE_PHASE_1_FOLLOWUP_MOVES
                    [*self.moves.last().unwrap() as u8 as usize]
            }
        };

        for mv in possible_moves {
            let new_cube = cube.do_move(*mv);

            // Check for solutions
            if new_cube.is_phase_solved() {
                // Only proceed at the requested depth, we don't want to repeat earlier searches
                if depth == 1 {
                    // Translate cube state into phase 2 index form
                    self.moves.push(*mv);
                    let mut pieces = self.initial_state.clone();
                    for mv in &self.moves {
                        pieces.do_move(*mv);
                    }
                    let cube = Phase2IndexCube::new(&new_cube, &pieces);

                    // Search for phase 2 solution using iterative deepening. Do not go beyond the maximum
                    // number of moves for the whole solve.
                    let mut depth = 0;
                    while depth < self.max_moves - self.moves.len() {
                        if self.search_phase_2(cube, depth) {
                            break;
                        }
                        depth += 1;
                    }

                    self.moves.pop();
                    if !self.optimal && self.best_solution.is_some() {
                        break;
                    }
                }
                continue;
            }

            if depth == 1 {
                continue;
            }

            // Check prune tables to see if a solution to this phase is impossible within the
            // given search depth
            if CombinedOrientationPruneTable::get(
                new_cube.corner_orientation,
                new_cube.edge_orientation,
            ) >= depth
            {
                continue;
            }
            if CornerOrientationEdgeSlicePruneTable::get(
                new_cube.corner_orientation,
                new_cube.equatorial_edge_slice,
            ) >= depth
            {
                continue;
            }
            if EdgeOrientationPruneTable::get(
                new_cube.edge_orientation,
                new_cube.equatorial_edge_slice,
            ) >= depth
            {
                continue;
            }
            if self.moves.len()
                + Phase1CornerPermutationPruneTable::get(new_cube.corner_permutation)
                >= self.max_moves
            {
                continue;
            }

            // Proceed further into phase 1
            self.moves.push(*mv);
            self.search_phase_1(new_cube, depth - 1);
            self.moves.pop();

            if !self.optimal && self.best_solution.is_some() {
                break;
            }
            if self.moves.len() + 1 >= self.max_moves {
                break;
            }
        }
    }

    fn search_phase_2(&mut self, cube: Phase2IndexCube, depth: usize) -> bool {
        // Check for solution
        if cube.is_phase_solved() {
            // Cube is solved, update best solution and stop this search path
            if self.best_solution.is_none()
                || self.moves.len() < self.best_solution.as_ref().unwrap().len()
            {
                self.best_solution = Some(self.moves.clone());
                self.max_moves = self.moves.len() - 1;
            }
            return true;
        } else if depth == 0 {
            return false;
        }

        // Check prune tables to see if it is possible to solve within the given depth
        if CornerEdgePermutationPruneTable::get(
            cube.corner_permutation,
            cube.equatorial_edge_permutation,
        ) > depth
        {
            return false;
        }
        if Phase2EdgePermutationPruneTable::get(
            cube.edge_permutation,
            cube.equatorial_edge_permutation,
        ) > depth
        {
            return false;
        }

        // Need to go deeper. Iterate through the possible moves.
        let possible_moves = if self.moves.len() == 0 {
            crate::tables::solve::CUBE3_POSSIBLE_PHASE_2_MOVES
        } else {
            crate::tables::solve::CUBE3_POSSIBLE_PHASE_2_FOLLOWUP_MOVES
                [*self.moves.last().unwrap() as u8 as usize]
        };
        for mv in possible_moves {
            self.moves.push(*mv);

            // Use move tables to transition to the next state for this move
            let new_cube = cube.do_move(*mv);

            // Proceed further into phase 2
            if self.search_phase_2(new_cube, depth - 1) {
                self.moves.pop();
                return true;
            }

            self.moves.pop();
        }

        false
    }

    fn solve(mut self) -> Option<Vec<Move>> {
        // If already solved, solution is zero moves
        if self.initial_state.is_solved() {
            return Some(Vec::new());
        }

        let cube = Phase1IndexCube::new(&self.initial_state);

        if cube.is_phase_solved() {
            // Phase 1 is already solved, translate cube state into phase 2 index form
            let cube = Phase2IndexCube::new(&cube, &self.initial_state);

            // Search for phase 2 solution using iterative deepening. Do not go beyond the maximum
            // number of moves for the whole solve.
            let mut depth = 1;
            while depth <= self.max_moves {
                if self.search_phase_2(cube, depth) {
                    break;
                }
                depth += 1;
            }
        } else {
            let mut depth = 1;
            while depth <= Cube3x3x3::MAX_PHASE_1_MOVES && depth <= self.max_moves {
                self.search_phase_1(cube, depth);
                depth += 1;
            }
        }

        self.best_solution
    }
}

impl Cube3x3x3 {
    pub const CORNER_ORIENTATION_INDEX_COUNT: usize = 2187; // 3**7
    pub const CORNER_PERMUTATION_INDEX_COUNT: usize = 40320; // 8!
    pub const EDGE_ORIENTATION_INDEX_COUNT: usize = 2048; // 2**11
    pub const PHASE_2_EDGE_PERMUTATION_INDEX_COUNT: usize = 40320; // 8!
    pub const EDGE_SLICE_INDEX_COUNT: usize = n_choose_k(12, 4);
    pub const PHASE_2_EQUATORIAL_EDGE_PERMUTATION_INDEX_COUNT: usize = 24; // 4!

    const MAX_PHASE_1_MOVES: usize = 12;
    const MAX_PHASE_2_MOVES: usize = 18;
    pub const MAX_SOLUTION_MOVES: usize = Self::MAX_PHASE_1_MOVES + Self::MAX_PHASE_2_MOVES;

    pub fn from_corners_and_edges(corners: [CornerPiece; 8], edges: [EdgePiece3x3x3; 12]) -> Self {
        Self { corners, edges }
    }

    /// Gets the piece at a given corner
    pub fn corner_piece(&self, corner: Corner) -> CornerPiece {
        self.corners[corner as u8 as usize]
    }

    /// Gets the piece at a given edge
    pub fn edge_piece(&self, edge: Edge3x3x3) -> EdgePiece3x3x3 {
        self.edges[edge as u8 as usize]
    }

    /// Index for the corner orientations is a simple base 3 integer representation. The
    /// zero index is the solved state. Note that the last corner is not represented in
    /// the index as its value is implicit (all corner orientations must add to a
    /// multiple of 3).
    pub fn corner_orientation_index(&self) -> u16 {
        let mut result = 0;
        for i in 0..7 {
            result = (result * 3) + self.corners[i].orientation as u16;
        }
        result
    }

    /// Index for the corner permutations is the representation of the state in the
    /// factorial number system (each digit in the number decreases in base, with the
    /// digits representing the index of the choice in the remaining possible choices).
    pub fn corner_permutation_index(&self) -> u16 {
        let mut result = 0;
        for i in 0..7 {
            // Get index in set of remaining options by checking how many of the entries
            // are greater than this one (which is the index in the sorted list of
            // remaining options)
            let mut cur = 0;
            for j in i + 1..8 {
                if self.corners[i].piece as u8 > self.corners[j].piece as u8 {
                    cur += 1;
                }
            }
            result = (result + cur) * (7 - i as u16);
        }
        result
    }

    /// Index for the edge orientations is a simple binary integer representation. The
    /// zero index is the solved state. Note that the last edge is not represented in
    /// the index as its value is implicit (all edge orientations must add to a
    /// multiple of 2).
    pub fn edge_orientation_index(&self) -> u16 {
        let mut result = 0;
        for i in 0..11 {
            result = (result * 2) + self.edges[i].orientation as u16;
        }
        result
    }

    /// Index for the edge permutations is the representation of the state in the
    /// factorial number system (each digit in the number decreases in base, with the
    /// digits representing the index of the choice in the remaining possible choices).
    /// This is the phase 2 edge permutation index, which does not include the edges
    /// in the equatorial slice (significantly reducing the count).
    pub fn phase_2_edge_permutation_index(&self) -> u16 {
        let mut result = 0;
        for i in 0..7 {
            // Get index in set of remaining options by checking how many of the entries
            // are greater than this one (which is the index in the sorted list of
            // remaining options)
            let mut cur = 0;
            for j in i + 1..8 {
                if self.edges[i].piece as u8 > self.edges[j].piece as u8 {
                    cur += 1;
                }
            }
            result = (result + cur) * (7 - i as u16);
        }
        result
    }

    /// Find the positions of the edge pieces that belong in the equatorial slice. For this
    /// index, it does not matter what order these pieces are in, so which piece it is can
    /// be ignored. The four positions should be in sorted order so that the permutations
    /// of ordering do not matter. This allows the index to be generated using the
    /// combinatorial number system. Represent in a way that the equatorial slice members
    /// have position 0-3 when they are in the equatorial slice (this allows the zero
    /// index to represent the solved state).
    pub fn equatorial_edge_slice_index(&self) -> u16 {
        let mut edge_piece_pos: [usize; 4] = [0; 4];
        let mut j = 0;
        for i in 0..12 {
            if self.edges[(i + Edge3x3x3::FR as u8 as usize) % 12].piece as u8
                >= Edge3x3x3::FR as u8
                && self.edges[(i + Edge3x3x3::FR as u8 as usize) % 12].piece as u8
                    <= Edge3x3x3::BR as u8
            {
                edge_piece_pos[j] = i;
                j += 1;
            }
        }
        // Compute an index using the combinatorial number system. This will be an integer
        // between zero (solved) and NChooseK(12, 4).
        (n_choose_k(edge_piece_pos[0], 1)
            + n_choose_k(edge_piece_pos[1], 2)
            + n_choose_k(edge_piece_pos[2], 3)
            + n_choose_k(edge_piece_pos[3], 4)) as u16
    }

    /// This index is only valid for phase 2 (equatorial edge pieces are already in the slice
    /// but not necessarily in the proper places). Index for the edge permutations is the
    /// representation of the state in the factorial number system (each digit in the number
    /// decreases in base, with the digits representing the index of the choice in the
    /// remaining possible choices).
    pub fn phase_2_equatorial_edge_permutation_index(&self) -> u16 {
        let mut result = 0;
        for i in 0..3 {
            // Get index in set of remaining options by checking how many of the entries
            // are greater than this one (which is the index in the sorted list of
            // remaining options)
            let mut cur = 0;
            for j in i + 1..4 {
                if self.edges[i + Edge3x3x3::FR as u8 as usize].piece as u8
                    > self.edges[j + Edge3x3x3::FR as u8 as usize].piece as u8
                {
                    cur += 1;
                }
            }
            result = (result + cur) * (3 - i as u16);
        }
        result
    }

    /// Gets this cube state in face color format
    pub fn as_faces(&self) -> Cube3x3x3Faces {
        let mut faces = Cube3x3x3Faces::new();

        // Translate corner pieces into face colors
        for corner_idx in 0..8 {
            let piece = self.corners[corner_idx];
            for i in 0..3 {
                let dest = crate::tables::corner::CUBE3_CORNER_INDICIES[corner_idx][i];
                let src = crate::tables::corner::CUBE3_CORNER_INDICIES[piece.piece as u8 as usize]
                    [(i + 3 - piece.orientation as usize) % 3];
                let face = Cube3x3x3Faces::face_for_idx(src);
                faces.state[dest] = faces.state[Cube3x3x3Faces::idx(face, 1, 1)];
            }
        }

        // Translate edge pieces into face colors
        for edge_idx in 0..12 {
            let piece = self.edges[edge_idx];
            for i in 0..2 {
                let dest = crate::tables::table3x3x3::CUBE3_EDGE_INDICIES[edge_idx][i];
                let src = crate::tables::table3x3x3::CUBE3_EDGE_INDICIES
                    [piece.piece as u8 as usize][i ^ piece.orientation as usize];
                let face = Cube3x3x3Faces::face_for_idx(src);
                faces.state[dest] = faces.state[Cube3x3x3Faces::idx(face, 1, 1)];
            }
        }

        faces
    }
}

impl FaceRotation for Cube3x3x3 {
    fn rotate_wide(&mut self, face: CubeFace, dir: RotationDirection, _width: usize) {
        let face_idx = face as u8 as usize;
        let dir_idx = dir as u8 as usize;

        // Save existing cube state so that it can be looked up during rotation
        let old_corners = self.corners;
        let old_edges = self.edges;

        // Apply corner movement using lookup table
        for i in 0..4 {
            let (dest, src) =
                crate::tables::corner::CUBE_CORNER_PIECE_ROTATION[dir_idx][face_idx][i];
            self.corners[dest as u8 as usize] = CornerPiece {
                piece: old_corners[src.piece as u8 as usize].piece,
                orientation: (old_corners[src.piece as u8 as usize].orientation + src.orientation)
                    % 3,
            };
        }

        // Apply edge movement using lookup table
        for i in 0..4 {
            let (dest, src) =
                crate::tables::table3x3x3::CUBE3_EDGE_PIECE_ROTATION[dir_idx][face_idx][i];
            self.edges[dest as u8 as usize] = EdgePiece3x3x3 {
                piece: old_edges[src.piece as u8 as usize].piece,
                orientation: (old_edges[src.piece as u8 as usize].orientation ^ src.orientation),
            };
        }
    }
}

impl InitialCubeState for Cube3x3x3 {
    fn new() -> Self {
        let mut corners = [CornerPiece {
            piece: Corner::URF,
            orientation: 0,
        }; 8];
        for i in 0..8 {
            corners[i].piece = Corner::try_from(i as u8).unwrap();
        }

        let mut edges = [EdgePiece3x3x3 {
            piece: Edge3x3x3::UR,
            orientation: 0,
        }; 12];
        for i in 0..12 {
            edges[i].piece = Edge3x3x3::try_from(i as u8).unwrap();
        }

        Self { corners, edges }
    }

    fn sourced_random<T: RandomSource>(rng: &mut T) -> Self {
        let mut cube = Self::new();

        // Randomize the corner pieces
        for i in 0..7 {
            let n = rng.next(8) as usize;
            if i != n {
                // Must swap two corners at a time to avoid parity violation
                cube.corners.swap(i, n);
                cube.corners.swap(6, 7);
            }
        }

        // Randomize the edge pieces
        for i in 0..11 {
            let n = rng.next(12) as usize;
            if i != n {
                // Must swap two edges at a time to avoid parity violation
                cube.edges.swap(i, n);
                cube.edges.swap(10, 11);
            }
        }

        // Randomize the corner orientations
        let mut corner_orientation_sum = 0;
        for i in 0..7 {
            cube.corners[i].orientation = rng.next(3) as u8;
            corner_orientation_sum += cube.corners[i].orientation;
        }

        // Randomize the edge orientations
        let mut edge_orientation_sum = 0;
        for i in 0..11 {
            cube.edges[i].orientation = rng.next(2) as u8;
            edge_orientation_sum += cube.edges[i].orientation;
        }

        // Make sure all corner orientations add up to a multiple of 3 (otherwise it is not solvable)
        cube.corners[7].orientation = (3 - (corner_orientation_sum % 3)) % 3;

        // Make sure all edge orientations add up to a multiple of 2 (otherwise it is not solvable)
        cube.edges[11].orientation = edge_orientation_sum & 1;

        cube
    }
}

impl Cube for Cube3x3x3 {
    fn is_solved(&self) -> bool {
        // Check corners
        for i in 0..8 {
            let correct_piece = CornerPiece {
                piece: Corner::try_from(i as u8).unwrap(),
                orientation: 0,
            };
            if self.corners[i] != correct_piece {
                return false;
            }
        }

        // Check edges
        for i in 0..12 {
            let correct_piece = EdgePiece3x3x3 {
                piece: Edge3x3x3::try_from(i as u8).unwrap(),
                orientation: 0,
            };
            if self.edges[i] != correct_piece {
                return false;
            }
        }
        true
    }

    fn do_move(&mut self, mv: Move) {
        self.rotate_counted(mv.face(), mv.rotation());
    }

    fn size(&self) -> usize {
        3
    }

    fn colors(&self) -> BTreeMap<CubeFace, Vec<Vec<Color>>> {
        self.as_faces().colors()
    }

    #[cfg(not(feature = "no_solver"))]
    fn solve(&self) -> Option<Vec<Move>> {
        Solver::new(self, true).solve()
    }

    #[cfg(not(feature = "no_solver"))]
    fn solve_fast(&self) -> Option<Vec<Move>> {
        Solver::new(self, false).solve()
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn dyn_clone(&self) -> Box<dyn Cube> {
        Box::new(self.clone())
    }
}

impl Cube3x3x3Faces {
    /// Create a cube state from a color array. The ordering of the array is the faces
    /// in order of the `Face` enumeration, with 9 elements per face. Each face is stored
    /// from top to bottom in row major order, with columns left to right.
    pub fn from_colors(state: [Color; 6 * 9]) -> Self {
        Self { state }
    }

    pub(crate) const fn face_start(face: CubeFace) -> usize {
        face as u8 as usize * 9
    }

    pub(crate) const fn face_offset(row: usize, col: usize) -> usize {
        (row * 3) + col
    }

    pub(crate) const fn idx(face: CubeFace, row: usize, col: usize) -> usize {
        Self::face_start(face) + Self::face_offset(row, col)
    }

    pub(crate) fn face_for_idx(idx: usize) -> CubeFace {
        CubeFace::try_from((idx / 9) as u8).unwrap()
    }

    /// Gets the color for a given place on the cube. For a given `face`, the `row` and
    /// `col` represent the zero-indexed position on the face to be accessed.
    pub fn color(&self, face: CubeFace, row: usize, col: usize) -> Color {
        self.state[Self::idx(face, row, col)]
    }

    /// Gets the color for a given place on the cube. For a given `face`, the `row` and
    /// `col` represent the zero-indexed position on the face to be accessed.
    pub(crate) fn color_by_idx(&self, idx: usize) -> Color {
        self.state[idx]
    }

    /// Gets the color of a specific corner (there are three colors per corner)
    pub fn corner_color(&self, corner: Corner, idx: usize) -> Color {
        self.state[crate::tables::corner::CUBE3_CORNER_INDICIES[corner as u8 as usize][idx]]
    }

    /// Gets the color of a specific edge (there are two colors per edge)
    pub fn edge_color(&self, edge: Edge3x3x3, idx: usize) -> Color {
        self.state[crate::tables::table3x3x3::CUBE3_EDGE_INDICIES[edge as u8 as usize][idx]]
    }

    /// Gets this cube state in piece format
    pub fn as_pieces(&self) -> Cube3x3x3 {
        let mut pieces = Cube3x3x3::new();

        for corner_idx in 0..8 {
            let corner_colors: [Color; 3] = [
                self.corner_color(Corner::try_from(corner_idx).unwrap(), 0),
                self.corner_color(Corner::try_from(corner_idx).unwrap(), 1),
                self.corner_color(Corner::try_from(corner_idx).unwrap(), 2),
            ];
            // Find this corner piece and orientation
            for i in 0..8 {
                if corner_colors[0] == crate::tables::corner::CUBE_CORNER_COLORS[i][0]
                    && corner_colors[1] == crate::tables::corner::CUBE_CORNER_COLORS[i][1]
                    && corner_colors[2] == crate::tables::corner::CUBE_CORNER_COLORS[i][2]
                {
                    pieces.corners[corner_idx as usize] = CornerPiece {
                        piece: Corner::try_from(i as u8).unwrap(),
                        orientation: 0,
                    };
                    break;
                } else if corner_colors[1] == crate::tables::corner::CUBE_CORNER_COLORS[i][0]
                    && corner_colors[2] == crate::tables::corner::CUBE_CORNER_COLORS[i][1]
                    && corner_colors[0] == crate::tables::corner::CUBE_CORNER_COLORS[i][2]
                {
                    pieces.corners[corner_idx as usize] = CornerPiece {
                        piece: Corner::try_from(i as u8).unwrap(),
                        orientation: 1,
                    };
                    break;
                } else if corner_colors[2] == crate::tables::corner::CUBE_CORNER_COLORS[i][0]
                    && corner_colors[0] == crate::tables::corner::CUBE_CORNER_COLORS[i][1]
                    && corner_colors[1] == crate::tables::corner::CUBE_CORNER_COLORS[i][2]
                {
                    pieces.corners[corner_idx as usize] = CornerPiece {
                        piece: Corner::try_from(i as u8).unwrap(),
                        orientation: 2,
                    };
                    break;
                }
            }
        }

        for edge_idx in 0..12 {
            let edge_colors: [Color; 2] = [
                self.edge_color(Edge3x3x3::try_from(edge_idx).unwrap(), 0),
                self.edge_color(Edge3x3x3::try_from(edge_idx).unwrap(), 1),
            ];
            // Find this edge piece and orientation
            for i in 0..12 {
                if edge_colors[0] == crate::tables::table3x3x3::CUBE3_EDGE_COLORS[i][0]
                    && edge_colors[1] == crate::tables::table3x3x3::CUBE3_EDGE_COLORS[i][1]
                {
                    pieces.edges[edge_idx as usize] = EdgePiece3x3x3 {
                        piece: Edge3x3x3::try_from(i as u8).unwrap(),
                        orientation: 0,
                    };
                    break;
                } else if edge_colors[1] == crate::tables::table3x3x3::CUBE3_EDGE_COLORS[i][0]
                    && edge_colors[0] == crate::tables::table3x3x3::CUBE3_EDGE_COLORS[i][1]
                {
                    pieces.edges[edge_idx as usize] = EdgePiece3x3x3 {
                        piece: Edge3x3x3::try_from(i as u8).unwrap(),
                        orientation: 1,
                    };
                    break;
                }
            }
        }

        pieces
    }
}

impl FaceRotation for Cube3x3x3Faces {
    fn rotate_wide(&mut self, face: CubeFace, dir: RotationDirection, _width: usize) {
        let face_idx = face as u8 as usize;
        let dir_idx = dir as u8 as usize;

        // Rotate colors on face itself
        let mut rotated_colors: [Color; 9] = [Color::White; 9];
        for i in 0..9 {
            rotated_colors[i] = self.state[Self::face_start(face)
                + crate::tables::table3x3x3::CUBE3_FACE_ROTATION[dir_idx][i]];
        }
        for i in 0..9 {
            self.state[Self::face_start(face) + i] = rotated_colors[i];
        }

        // Collect colors on edges and corners
        let mut adjacent_edge_colors: [Color; 4] = [Color::White; 4];
        let mut adjacent_corner_colors: [[Color; 2]; 4] = [[Color::White; 2]; 4];
        for i in 0..4 {
            adjacent_edge_colors[i] =
                self.state[crate::tables::table3x3x3::CUBE3_EDGE_ADJACENCY[face_idx][i]];
            adjacent_corner_colors[i][0] =
                self.state[crate::tables::corner::CUBE3_CORNER_ADJACENCY[face_idx][i][0]];
            adjacent_corner_colors[i][1] =
                self.state[crate::tables::corner::CUBE3_CORNER_ADJACENCY[face_idx][i][1]];
        }

        // Rotate colors on edges and corners
        for i in 0..4 {
            let j = crate::tables::table3x3x3::CUBE3_EDGE_ROTATION[dir_idx][i];
            let k = crate::tables::corner::CUBE_CORNER_ROTATION[dir_idx][i];
            self.state[crate::tables::table3x3x3::CUBE3_EDGE_ADJACENCY[face_idx][j]] =
                adjacent_edge_colors[i];
            self.state[crate::tables::corner::CUBE3_CORNER_ADJACENCY[face_idx][k][0]] =
                adjacent_corner_colors[i][0];
            self.state[crate::tables::corner::CUBE3_CORNER_ADJACENCY[face_idx][k][1]] =
                adjacent_corner_colors[i][1];
        }
    }
}

impl InitialCubeState for Cube3x3x3Faces {
    fn new() -> Self {
        let mut state = [Color::White; 6 * 9];
        for i in 0..9 {
            state[Self::face_start(CubeFace::Top) + i] = Color::White;
            state[Self::face_start(CubeFace::Front) + i] = Color::Green;
            state[Self::face_start(CubeFace::Right) + i] = Color::Red;
            state[Self::face_start(CubeFace::Back) + i] = Color::Blue;
            state[Self::face_start(CubeFace::Left) + i] = Color::Orange;
            state[Self::face_start(CubeFace::Bottom) + i] = Color::Yellow;
        }
        Self { state }
    }

    fn sourced_random<T: RandomSource>(rng: &mut T) -> Self {
        Cube3x3x3::sourced_random(rng).as_faces()
    }
}

impl Cube for Cube3x3x3Faces {
    fn is_solved(&self) -> bool {
        for face in 0..6 {
            let face = CubeFace::try_from(face).unwrap();
            for i in 0..9 {
                // All colors on a face must match center
                if self.state[Self::face_start(face) + i] != self.state[Self::idx(face, 1, 1)] {
                    return false;
                }
            }
        }
        true
    }

    fn do_move(&mut self, mv: Move) {
        self.rotate_counted(mv.face(), mv.rotation());
    }

    fn size(&self) -> usize {
        3
    }

    fn colors(&self) -> BTreeMap<CubeFace, Vec<Vec<Color>>> {
        let mut result = BTreeMap::new();
        for face in &[
            CubeFace::Top,
            CubeFace::Front,
            CubeFace::Right,
            CubeFace::Back,
            CubeFace::Left,
            CubeFace::Bottom,
        ] {
            let mut rows = Vec::new();
            for row in 0..3 {
                let mut cols = Vec::new();
                for col in 0..3 {
                    cols.push(self.color(*face, row, col));
                }
                rows.push(cols);
            }
            result.insert(*face, rows);
        }
        result
    }

    #[cfg(not(feature = "no_solver"))]
    fn solve(&self) -> Option<Vec<Move>> {
        self.as_pieces().solve()
    }

    #[cfg(not(feature = "no_solver"))]
    fn solve_fast(&self) -> Option<Vec<Move>> {
        self.as_pieces().solve_fast()
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn dyn_clone(&self) -> Box<dyn Cube> {
        Box::new(self.clone())
    }
}

impl std::fmt::Display for Cube3x3x3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.as_faces().fmt(f)
    }
}

impl std::fmt::Display for Cube3x3x3Faces {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut debug_state: [[char; 13]; 9] = [[' '; 13]; 9];
        const FACE_X: [usize; 6] = [3, 3, 6, 9, 0, 3];
        const FACE_Y: [usize; 6] = [0, 3, 3, 3, 3, 6];
        for face_idx in 0..6 {
            for row in 0..3 {
                for col in 0..3 {
                    let ch = match self.state
                        [Self::idx(CubeFace::try_from(face_idx).unwrap(), row, col)]
                    {
                        Color::White => 'W',
                        Color::Green => 'G',
                        Color::Red => 'R',
                        Color::Blue => 'B',
                        Color::Orange => 'O',
                        Color::Yellow => 'Y',
                    };
                    debug_state[FACE_Y[face_idx as usize] + row][FACE_X[face_idx as usize] + col] =
                        ch;
                }
            }
        }
        for row in 0..9 {
            let s: String = debug_state[row].iter().collect();
            write!(f, "{}\n", s)?;
        }
        Ok(())
    }
}

/// Generates a random scramble
#[cfg(not(feature = "no_solver"))]
pub fn scramble_3x3x3() -> Vec<Move> {
    let state = Cube3x3x3::random();
    let solution = state.solve().unwrap();
    solution.inverse()
}

/// Generates a random scramble very fast, but with more moves required than normal
#[cfg(not(feature = "no_solver"))]
pub fn scramble_3x3x3_fast() -> Vec<Move> {
    let state = Cube3x3x3::random();
    let solution = state.solve_fast().unwrap();
    solution.inverse()
}
