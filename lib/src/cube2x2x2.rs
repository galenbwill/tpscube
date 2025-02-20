use crate::{
    Color, Corner, CornerPiece, Cube, CubeFace, FaceRotation, InitialCubeState, Move, RandomSource,
    RotationDirection,
};
use std::collections::BTreeMap;
use std::convert::TryFrom;

#[cfg(not(feature = "no_solver"))]
use crate::common::{
    CornerOrientationMoveTable, CornerOrientationPruneTable, CornerPermutationMoveTable,
    CornerPermutationPruneTable, MoveSequence,
};

#[derive(Debug, PartialEq, Eq, Clone)]
/// A 2x2x2 cube represented in piece format (optimal for computational algorithms).
pub struct Cube2x2x2 {
    corners: [CornerPiece; 8],
}

#[derive(Debug, PartialEq, Eq, Clone)]
/// A 2x2x2 cube represented in face color format (easy to use, matches visuals).
pub struct Cube2x2x2Faces {
    state: [Color; 6 * 4],
}

#[cfg(not(feature = "no_solver"))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct IndexCube {
    corner_orientation: u16,
    corner_permutation: u16,
}

#[cfg(not(feature = "no_solver"))]
impl IndexCube {
    fn new(pieces: &Cube2x2x2) -> Self {
        Self {
            corner_orientation: pieces.corner_orientation_index(),
            corner_permutation: pieces.corner_permutation_index(),
        }
    }

    fn do_move(&self, mv: Move) -> Self {
        Self {
            corner_orientation: CornerOrientationMoveTable::get(self.corner_orientation, mv),
            corner_permutation: CornerPermutationMoveTable::get(self.corner_permutation, mv),
        }
    }

    fn is_solved(&self) -> bool {
        self.corner_orientation == 0 && self.corner_permutation == 0
    }
}

#[cfg(not(feature = "no_solver"))]
struct Solver {
    initial_state: Cube2x2x2,
    moves: Vec<Move>,
    max_moves: usize,
    solution: Option<Vec<Move>>,
}

#[cfg(not(feature = "no_solver"))]
impl Solver {
    fn new(cube: &Cube2x2x2) -> Self {
        Self {
            initial_state: cube.clone(),
            moves: Vec::new(),
            max_moves: Cube2x2x2::MAX_SOLUTION_MOVES,
            solution: None,
        }
    }

    fn search(&mut self, cube: IndexCube, depth: usize) {
        // Need to go deeper. Iterate through the possible moves.
        let possible_moves = if self.moves.len() == 0 {
            crate::tables::solve::CUBE2_POSSIBLE_MOVES
        } else {
            crate::tables::solve::CUBE2_POSSIBLE_FOLLOWUP_MOVES
                [*self.moves.last().unwrap() as u8 as usize]
        };

        for mv in possible_moves {
            let new_cube = cube.do_move(*mv);

            // Check for solutions
            if new_cube.is_solved() {
                let mut moves = self.moves.clone();
                moves.push(*mv);
                self.solution = Some(moves);
                break;
            }

            if depth == 1 {
                continue;
            }

            // Check prune tables to see if a solution is impossible within the given search depth
            if CornerOrientationPruneTable::get(new_cube.corner_orientation) >= depth {
                continue;
            }
            if CornerPermutationPruneTable::get(new_cube.corner_permutation) >= depth {
                continue;
            }

            // Proceed further into phase 1
            self.moves.push(*mv);
            self.search(new_cube, depth - 1);
            self.moves.pop();

            if self.solution.is_some() {
                break;
            }
        }
    }

    fn solve(mut self) -> Option<Vec<Move>> {
        // If already solved, solution is zero moves
        if self.initial_state.is_solved() {
            return Some(Vec::new());
        }

        let cube = IndexCube::new(&self.initial_state);

        let mut depth = 1;
        while depth <= self.max_moves && self.solution.is_none() {
            self.search(cube, depth);
            depth += 1;
        }

        self.solution
    }
}

impl Cube2x2x2 {
    pub const MAX_SOLUTION_MOVES: usize = 14;

    pub fn from_corners(corners: [CornerPiece; 8]) -> Self {
        Self { corners }
    }

    /// Gets the piece at a given corner
    pub fn corner_piece(&self, corner: Corner) -> CornerPiece {
        self.corners[corner as u8 as usize]
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

    /// Gets this cube state in face color format
    pub fn as_faces(&self) -> Cube2x2x2Faces {
        let mut faces = Cube2x2x2Faces::new();

        // Translate corner pieces into face colors
        for corner_idx in 0..8 {
            let piece = self.corners[corner_idx];
            for i in 0..3 {
                let dest = crate::tables::corner::CUBE2_CORNER_INDICIES[corner_idx][i];
                let src = crate::tables::corner::CUBE2_CORNER_INDICIES[piece.piece as u8 as usize]
                    [(i + 3 - piece.orientation as usize) % 3];
                let face = Cube2x2x2Faces::face_for_idx(src);
                faces.state[dest] = face.color();
            }
        }

        faces
    }
}

impl FaceRotation for Cube2x2x2 {
    fn rotate_wide(&mut self, face: CubeFace, dir: RotationDirection, _width: usize) {
        let face_idx = face as u8 as usize;
        let dir_idx = dir as u8 as usize;

        // Save existing cube state so that it can be looked up during rotation
        let old_corners = self.corners;

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
    }
}

impl InitialCubeState for Cube2x2x2 {
    fn new() -> Self {
        let mut corners = [CornerPiece {
            piece: Corner::URF,
            orientation: 0,
        }; 8];
        for i in 0..8 {
            corners[i].piece = Corner::try_from(i as u8).unwrap();
        }

        Self { corners }
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

        // Randomize the corner orientations
        let mut corner_orientation_sum = 0;
        for i in 0..7 {
            cube.corners[i].orientation = rng.next(3) as u8;
            corner_orientation_sum += cube.corners[i].orientation;
        }

        // Make sure all corner orientations add up to a multiple of 3 (otherwise it is not solvable)
        cube.corners[7].orientation = (3 - (corner_orientation_sum % 3)) % 3;

        cube
    }
}

impl Cube for Cube2x2x2 {
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
        true
    }

    fn do_move(&mut self, mv: Move) {
        self.rotate_counted(mv.face(), mv.rotation());
    }

    fn size(&self) -> usize {
        2
    }

    fn colors(&self) -> BTreeMap<CubeFace, Vec<Vec<Color>>> {
        self.as_faces().colors()
    }

    #[cfg(not(feature = "no_solver"))]
    fn solve(&self) -> Option<Vec<Move>> {
        Solver::new(self).solve()
    }

    #[cfg(not(feature = "no_solver"))]
    fn solve_fast(&self) -> Option<Vec<Move>> {
        Solver::new(self).solve()
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn dyn_clone(&self) -> Box<dyn Cube> {
        Box::new(self.clone())
    }
}

impl Cube2x2x2Faces {
    /// Create a cube state from a color array. The ordering of the array is the faces
    /// in order of the `Face` enumeration, with 4 elements per face. Each face is stored
    /// from top to bottom in row major order, with columns left to right.
    pub fn from_colors(state: [Color; 6 * 4]) -> Self {
        Self { state }
    }

    pub(crate) const fn face_start(face: CubeFace) -> usize {
        face as u8 as usize * 4
    }

    pub(crate) const fn face_offset(row: usize, col: usize) -> usize {
        (row * 2) + col
    }

    pub(crate) const fn idx(face: CubeFace, row: usize, col: usize) -> usize {
        Self::face_start(face) + Self::face_offset(row, col)
    }

    pub(crate) fn face_for_idx(idx: usize) -> CubeFace {
        CubeFace::try_from((idx / 4) as u8).unwrap()
    }

    /// Gets the color for a given place on the cube. For a given `face`, the `row` and
    /// `col` represent the zero-indexed position on the face to be accessed.
    pub fn color(&self, face: CubeFace, row: usize, col: usize) -> Color {
        self.state[Self::idx(face, row, col)]
    }

    /// Gets the color of a specific corner (there are three colors per corner)
    pub fn corner_color(&self, corner: Corner, idx: usize) -> Color {
        self.state[crate::tables::corner::CUBE2_CORNER_INDICIES[corner as u8 as usize][idx]]
    }

    /// Gets this cube state in piece format
    pub fn as_pieces(&self) -> Cube2x2x2 {
        let mut pieces = Cube2x2x2::new();

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

        pieces
    }
}

impl FaceRotation for Cube2x2x2Faces {
    fn rotate_wide(&mut self, face: CubeFace, dir: RotationDirection, _width: usize) {
        let face_idx = face as u8 as usize;
        let dir_idx = dir as u8 as usize;

        // Rotate colors on face itself
        let mut rotated_colors: [Color; 9] = [Color::White; 9];
        for i in 0..4 {
            rotated_colors[i] = self.state[Self::face_start(face)
                + crate::tables::table2x2x2::CUBE2_FACE_ROTATION[dir_idx][i]];
        }
        for i in 0..4 {
            self.state[Self::face_start(face) + i] = rotated_colors[i];
        }

        // Collect colors on corners
        let mut adjacent_corner_colors: [[Color; 2]; 4] = [[Color::White; 2]; 4];
        for i in 0..4 {
            adjacent_corner_colors[i][0] =
                self.state[crate::tables::corner::CUBE2_CORNER_ADJACENCY[face_idx][i][0]];
            adjacent_corner_colors[i][1] =
                self.state[crate::tables::corner::CUBE2_CORNER_ADJACENCY[face_idx][i][1]];
        }

        // Rotate colors on corners
        for i in 0..4 {
            let j = crate::tables::corner::CUBE_CORNER_ROTATION[dir_idx][i];
            self.state[crate::tables::corner::CUBE2_CORNER_ADJACENCY[face_idx][j][0]] =
                adjacent_corner_colors[i][0];
            self.state[crate::tables::corner::CUBE2_CORNER_ADJACENCY[face_idx][j][1]] =
                adjacent_corner_colors[i][1];
        }
    }
}

impl InitialCubeState for Cube2x2x2Faces {
    fn new() -> Self {
        let mut state = [Color::White; 6 * 4];
        for i in 0..4 {
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
        Cube2x2x2::sourced_random(rng).as_faces()
    }
}

impl Cube for Cube2x2x2Faces {
    fn is_solved(&self) -> bool {
        for face in 0..6 {
            let face = CubeFace::try_from(face).unwrap();
            for i in 0..4 {
                // All colors on a face must match face color
                if self.state[Self::face_start(face) + i] != face.color() {
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
        2
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
            for row in 0..2 {
                let mut cols = Vec::new();
                for col in 0..2 {
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

impl std::fmt::Display for Cube2x2x2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.as_faces().fmt(f)
    }
}

impl std::fmt::Display for Cube2x2x2Faces {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut debug_state: [[char; 9]; 6] = [[' '; 9]; 6];
        const FACE_X: [usize; 6] = [2, 2, 4, 6, 0, 2];
        const FACE_Y: [usize; 6] = [0, 2, 2, 2, 2, 4];
        for face_idx in 0..6 {
            for row in 0..2 {
                for col in 0..2 {
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
        for row in 0..6 {
            let s: String = debug_state[row].iter().collect();
            write!(f, "{}\n", s)?;
        }
        Ok(())
    }
}

/// Generates a random scramble
#[cfg(not(feature = "no_solver"))]
pub fn scramble_2x2x2() -> Vec<Move> {
    let state = Cube2x2x2::random();
    let solution = state.solve().unwrap();
    solution.inverse()
}
