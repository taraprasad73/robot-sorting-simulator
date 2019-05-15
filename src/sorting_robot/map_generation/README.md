# Map Generation Package

## Main constraints followed while creating the map
 - Directions should alternate on neighboring roads. Left should follow right and vice-versa. Up should follow down and vice-versa.
 - Motion should happen on left when on highways.
 - No right turns are allowed when on highways.
 - Lanes can only be switched on highway-highway intersections
 - There can only be one pickup area per block.
 - There can only be one charging station and queue per block.
 - The map should be symmetrical horizontally.

## Input arguments
All these arguments have appropriate default values and hence are optional. The values of the arguments should follow the constraints given in the next section.
~~~~
  -r                             total number of rows
  -c                             total number of columns
  -m                             number of rows in a block
  -n                             number of columns in a block
  --pickup-rows                  number of rows in the pickup area
  --pickup-columns               number of columns in the pickup area
  --charging-rows                number of rows in the charging area
  -p                             1-indexed row number of first horizontal highway
  -q                             1-indexed column number of first vertical highway
  --is-not-symmetric             boolean which tells whether to keep the grid symmetric or not
~~~~

## Constraints on the input
    - m >= 5 and m % 4 == 1
    - n >= 5 and n % 4 == 1
    - r >= m + 4
    - c >= n + 4
    - 1 <= p <= r - m - 3
    - 1 <= q <= c - n - 3
    for  symmetrical map:
        - p = 1
        - q = 1
        - r = (m + 2) * k + 2, for k >= 1
        - c = (n + 2) * k + 2, for k >= 1

## map_configuration.npy
It is a dict containing the following keys
  - 'grid'                           a 2D grid of _Cell_ objects
  - 'pickups'                         a dict containing the starting and ending position of each pickup
  - 'pickup_queue_size'               length of each pickup queue
  - 'num_rows'                        number of rows in the grid
  - 'num_columns'                     number of columns in the grid
  - 'cell_length_in_meters'           length of each cell in real world

## Cell Class
- __row__ row number (0-indexed) of the cell
- __col__ column number (0-indexed) of the cell
- __cellType__ a member of the enum _CellType_
- __allowedTurns__ a set containing elements of the enum class _Turn_. It tells the turns that are allowed to execute on that cell
- __directions__ a set containing elements of the enum class _Direction_. It tells the directions from which one can go from that cell to its neighbouring cell.
- __isObstacle__ a cell where a robot can never enter into. These cells are either of the following types:
  - RESTRICTED_AREA 
  - PARCEL_BIN
  - PICKUP_AREA

## CellType Enum
 - RESTRICTED_AREA
 - PARCEL_BIN
 - ONE_WAY_ROAD_ON_STREET
 - ONE_WAY_ROAD_ON_HIGHWAY
 - STREET_STREET_INTERSECTION
 - HIGHWAY_HIGHWAY_INTERSECTION
 - HIGHWAY_STREET_INTERSECTION
 - HIGHWAY_STREET_FORK
 - PICKUP_AREA
 - TOP_PICKUP_LANES
 - BOTTOM_PICKUP_LANES
 - PICKUP_QUEUE_START
 - PICKUP_QUEUE_FINISH
 - CHARGING_AREA
 - CHARGING_LANES