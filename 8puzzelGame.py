import matplotlib.pyplot as plt
import heapq
import random
import time
start_time = time.time()

# Define the possible moves
moves = {
    0: [1, 3],
    1: [0, 2, 4],
    2: [1, 5],
    3: [0, 4, 6],
    4: [1, 3, 5, 7],
    5: [2, 4, 8],
    6: [3, 7],
    7: [4, 6, 8],
    8: [5, 7]
}

# Define the misplaced tiles heuristic


def misplaced_tiles(state):
    goal_state = [1, 2, 3, 4, 5, 6, 7, 8, 0]
    state = state + [0] * (9 - len(state))
    count = 0
    for i in range(9):
        if state[i] != goal_state[i]:
            count += 1
    return count

# Define the Manhattan distance heuristic


def manhattan_distance(state):
    goal_state = [1, 2, 3, 4, 5, 6, 7, 8, 0]
    distance = 0
    for i in range(len(state)):
        if state[i] != 0:
            goal_index = goal_state.index(state[i])
            distance += abs(i // 3 - goal_index // 3) + \
                abs(i % 3 - goal_index % 3)
    return distance

# A* search algorithm


def astar_search(initial_state, heuristic):
    open_list = []
    closed_list = set()

    if not initial_state:
        initial_state = [1, 2, 3, 4, 5, 6, 7, 8, 0]

    start_node = (heuristic(initial_state), 0, initial_state)
    heapq.heappush(open_list, start_node)

    nodes_generated = 0
    while open_list:
        nodes_generated += 1
        _, moves_count, current_state = heapq.heappop(open_list)

        if current_state == goal_state:
            return nodes_generated

        closed_list.add(tuple(current_state))

        blank_index = current_state.index(0)
        for move in moves[blank_index]:
            new_state = current_state[:]
            new_state[blank_index], new_state[move] = new_state[move], new_state[blank_index]
            new_moves_count = moves_count + 1
            new_heuristic = heuristic(new_state)
            new_node = (new_heuristic + new_moves_count,
                        new_moves_count, new_state)

            if tuple(new_state) not in closed_list:
                heapq.heappush(open_list, new_node)

    return -1


# Calculate effective branching factor (EBF) for each depth and heuristic
misplaced_ebfs = {}
manhattan_ebfs = {}
depths = range(17, 27)

for depth in depths:
    misplaced_nodes = 0
    manhattan_nodes = 0
    for _ in range(50):
        # Generate a random initial state
        goal_state = [1, 2, 3, 4, 5, 6, 7, 8, 0]
        initial_state = random.sample(goal_state, 9)
        initial_state.append(0)

        # Calculate nodes generated for misplaced tiles heuristic
        misplaced_nodes += astar_search(initial_state, misplaced_tiles)

        # Calculate nodes generated for Manhattan distance heuristicس
        manhattan_nodes += astar_search(initial_state, manhattan_distance)

    misplaced_ebfs[depth] = misplaced_nodes / depth
    manhattan_ebfs[depth] = manhattan_nodes / depth
print("Total execution time:", time.time() - start_time, "seconds")
# Print the average EBF for each depth and heuristic
print("d\tEBF misplaced\tEBF manhattan:")
for depth in depths:
    print(f"{depth}\t{misplaced_ebfs[depth]*-
          1:.2f}\t{manhattan_ebfs[depth]*-1:.2f}")

# Plot the results
plt.plot(depths, misplaced_ebfs.values(), label="misplaced tiles")
plt.plot(depths, manhattan_ebfs.values(), label="Manhattan distance")
plt.xlabel("Depth")
plt.ylabel("Effective branching factor")
plt.legend()
plt.show()
