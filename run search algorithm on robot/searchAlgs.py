import numpy as np


def random_walk(cells_visited, env_map, old_pos, num_acts=4):
    x, y = np.add(old_pos, [1, 1])

    # boundaries
    left = y-1
    right = y+1
    top = x-1
    down = x+1

    decision_map = cells_visited[top:down+1, left:right+1].copy()
    decision_map_shape = np.shape(decision_map)
    # actions
    stay = [0, 0]
    go_right = [0, 1]
    go_left = [0, -1]
    go_up = [-1, 0]
    go_down = [1, 0]

    go_northeast = [-1, 1]
    go_northwest = [-1, -1]
    go_southeast = [1, 1]
    go_southwest = [1, -1]

    NORTH = 0
    NORTH_WEST = 45
    WEST = 90
    SOUTH_WEST = 135
    SOUTH = 180
    SOUTH_EAST = 225
    EAST = 270
    NORTH_EAST = 315

    actions_map = np.array([[go_northwest, go_up, go_northeast],
                            [go_left, stay, go_right],
                            [go_southwest, go_down, go_southeast]])

    rotation = np.array([[NORTH_WEST, NORTH, NORTH_EAST],
                         [WEST, 0, EAST],
                         [SOUTH_WEST, SOUTH, SOUTH_EAST]])

    ignore_pattern = np.array([[np.nan, 0, np.nan],
                               [0, np.nan, 0],
                               [np.nan, 0, np.nan]])
    if num_acts == 8:
        pass
    elif num_acts == 4:
        decision_map = np.add(decision_map, ignore_pattern)
    else:
        raise ValueError('Number of actions must be 4 or 8.')
    # use the following to break the ties between min values
    visit_flat_index = np.random.choice(np.flatnonzero(decision_map == 0))
    visit_index = np.unravel_index(visit_flat_index, decision_map_shape)
    action = actions_map[visit_index[0], visit_index[1]]
    orientation = rotation[visit_index[0], visit_index[1]]
    next_pos = np.add(old_pos, action)

    if env_map[next_pos[0], next_pos[1]] == 0:
        updated_pos = next_pos
    else:
        updated_pos = old_pos
    return updated_pos, orientation

def straight_walk(cells_visited, env_map, old_pos, num_acts=4):
    x, y = np.add(old_pos, [1, 1])

    # boundaries
    left = y-1
    right = y+1
    top = x-1
    down = x+1

    decision_map = cells_visited[top:down+1, left:right+1].copy()
    decision_map_shape = np.shape(decision_map)
    # actions
    stay = [0, 0]
    go_right = [0, 1]
    go_left = [0, -1]
    go_up = [-1, 0]
    go_down = [1, 0]

    go_northeast = [-1, 1]
    go_northwest = [-1, -1]
    go_southeast = [1, 1]
    go_southwest = [1, -1]

    NORTH = 0
    NORTH_WEST = 45
    WEST = 90
    SOUTH_WEST = 135
    SOUTH = 180
    SOUTH_EAST = 225
    EAST = 270
    NORTH_EAST = 315

    actions_map = np.array([[go_northwest, go_up, go_northeast],
                            [go_left, stay, go_right],
                            [go_southwest, go_down, go_southeast]])

    rotation = np.array([[NORTH_WEST, NORTH, NORTH_EAST],
                         [WEST, 0, EAST],
                         [SOUTH_WEST, SOUTH, SOUTH_EAST]])

    ignore_pattern = np.array([[np.nan, 0, np.nan],
                               [0, np.nan, 0],
                               [np.nan, 0, np.nan]])

    map_shape = np.shape(env_map)
    if np.sum(cells_visited[1:-1, 1:-1]) == (map_shape[0]*map_shape[1]):
        cells_visited[1:-1, 1:-1] = 0

    if num_acts == 8:
        pass
    elif num_acts == 4:
        decision_map = np.add(decision_map, ignore_pattern)
    else:
        raise ValueError('Number of actions must be 4 or 8.')
    decision_map_min_values = np.nanmin(decision_map)
    min_visit_flat_index = np.flatnonzero(decision_map == decision_map_min_values)
    min_visit_index = np.unravel_index(min_visit_flat_index[0], decision_map_shape)
    # use the following to break the ties between min values
    # min_visit_flat_index = np.random.choice(np.flatnonzero(decision_map == decision_map_min_values))
    # min_visit_index = np.unravel_index(min_visit_flat_index, decision_map_shape)
    action = actions_map[min_visit_index[0], min_visit_index[1]]
    orientation = rotation[min_visit_index[0], min_visit_index[1]]
    next_pos = np.add(old_pos, action)

    if env_map[next_pos[0], next_pos[1]] == 0:
        cells_visited[next_pos[0]+1, next_pos[1]+1] = 1
        updated_pos = next_pos
    else:
        cells_visited[old_pos[0]+1, old_pos[1]+1] = 1
        updated_pos = old_pos
    return updated_pos, orientation, cells_visited

def ant_colony(cells_visited, env_map, old_pos, num_acts=4):
    x, y = np.add(old_pos, [1, 1])

    # boundaries
    left = y-1
    right = y+1
    top = x-1
    down = x+1

    decision_map = cells_visited[top:down+1, left:right+1].copy()
    decision_map_shape = np.shape(decision_map)
    # actions
    stay = [0, 0]
    go_right = [0, 1]
    go_left = [0, -1]
    go_up = [-1, 0]
    go_down = [1, 0]

    go_northeast = [-1, 1]
    go_northwest = [-1, -1]
    go_southeast = [1, 1]
    go_southwest = [1, -1]

    NORTH = 0
    NORTH_WEST = 45
    WEST = 90
    SOUTH_WEST = 135
    SOUTH = 180
    SOUTH_EAST = 225
    EAST = 270
    NORTH_EAST = 315

    actions_map = np.array([[go_northwest, go_up, go_northeast],
                            [go_left, stay, go_right],
                            [go_southwest, go_down, go_southeast]])

    rotation = np.array([[NORTH_WEST, NORTH, NORTH_EAST],
                         [WEST, 0, EAST],
                         [SOUTH_WEST, SOUTH, SOUTH_EAST]])

    ignore_pattern = np.array([[np.nan, 0, np.nan],
                               [0, np.nan, 0],
                               [np.nan, 0, np.nan]])
    if num_acts == 8:
        pass
    elif num_acts == 4:
        decision_map = np.add(decision_map, ignore_pattern)
    else:
        raise ValueError('Number of actions must be 4 or 8.')
    decision_map_min_values = np.nanmin(decision_map)
    min_visit_flat_index = np.flatnonzero(decision_map == decision_map_min_values)
    min_visit_index = np.unravel_index(min_visit_flat_index[0], decision_map_shape)
    # use the following to break the ties between min values
    # min_visit_flat_index = np.random.choice(np.flatnonzero(decision_map == decision_map_min_values))
    # min_visit_index = np.unravel_index(min_visit_flat_index, decision_map_shape)
    action = actions_map[min_visit_index[0], min_visit_index[1]]
    orientation = rotation[min_visit_index[0], min_visit_index[1]]
    next_pos = np.add(old_pos, action)

    if env_map[next_pos[0], next_pos[1]] == 0:
        cells_visited[next_pos[0]+1, next_pos[1]+1] += 1
        updated_pos = next_pos
    else:
        cells_visited[old_pos[0]+1, old_pos[1]+1] += 1
        updated_pos = old_pos
    return updated_pos, orientation, cells_visited
