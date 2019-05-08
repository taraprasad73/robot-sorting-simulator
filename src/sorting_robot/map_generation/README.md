# Map Generation Package

## Main constraints followed while creating the map:
    - Directions should alternate. Left should follow right and vice-versa. Up should follow down and vice-versa.
    - Motion should happen on left when on highways.
    - No right turns are allowed when on highways.

## Input:
    - r           total number of rows
    - c           total number of columns
    - p           1-indexed row number of first horizontal highway
    - q           1-indexed column number of first vertical highway
    - m           rows in a block
    - n           columns in a block

## Constraints on the input:
    - m >= 5 and m % 4 == 1
    - n >= 5 and n % 4 == 1
    - r >= m + 4
    - c >= n + 4
    - 1 <= p <= r - m - 3
    - 1 <= q <= c - n - 3

## Output:
    - A 2D grid of cell objects, where each cell object contains the following fields:
        - row               row number (0-indexed) of the cell
        - col               column number (0-indexed) of the cell
        - isBin             a boolean which tells whether the cell is a bin or not
        - hasTurn           a boolean which tells whether a left turn can be made on that cell or not
        - directions        a set containing elements of the enum class - Direction. It tells the directions
                            from which one can enter the cell