from pyomo.environ import *

num_nodes = 39
nodes = range(num_nodes)

matrix_size = 39
distances = [[-1 for k in range(matrix_size)] for k in range(matrix_size)]

data = [
    (1, 2, 267),
    (1, 4, 252),
    (2, 3, 166),
    (2, 9, 69),
    (3, 4, 72),
    (3, 7, 116),
    (4, 5, 150),
    (5, 6, 52),
    (5, 29, 30),
    (6, 7, 57),
    (6, 30, 35),
    (7, 8, 82),
    (8, 9, 227),
    (8, 12, 126),
    (8, 37, 130),
    (9, 10, 84),
    (10, 11, 202),
    (11, 12, 152),
    (11, 14, 177),
    (12, 13, 149),
    (13, 15, 313),
    (13, 37, 99),
    (14, 15, 182),
    (15, 17, 251),
    (16, 17, 62),
    (16, 20, 349),
    (17, 18, 298),
    (18, 19, 441),
    (18, 39, 194),
    (19, 20, 107),
    (19, 23, 313),
    (20, 21, 106),
    (21, 22, 279),
    (21, 26, 529),
    (22, 23, 177),
    (22, 26, 340),
    (23, 25, 129),
    (24, 25, 53),
    (24, 28, 97),
    (25, 26, 199),
    (26, 27, 650),
    (27, 28, 414),
    (27, 29, 959),
    (28, 34, 101),
    (29, 30, 111),
    (30, 31, 79),
    (31, 32, 59),
    (31, 37, 52),
    (32, 33, 209),
    (32, 36, 56),
    (33, 34, 109),
    (33, 35, 37),
    (34, 39, 173),
    (35, 38, 79),
    (36, 37, 62),
    (38, 39, 221)
]

for i, j, dist in data:
    distances[i - 1][j - 1] = dist
    distances[j - 1][i - 1] = dist

#checking for multiple closed paths
def check_for_subtours(x_values):
    unvisited = list(nodes)
    subtours = []
    while unvisited:
        current_node = unvisited[0]
        subtour = [current_node]
        unvisited.remove(current_node)
        flag=0
        for j in nodes:
                if(x_values[(current_node, j)] == 1):
                    next_node=j
                    flag=1
                    break
        if(flag==0):
            continue
        while True:
            for j in nodes:
                if(x_values[(current_node, j)] == 1):
                    next_node=j
                    break
            if next_node == subtour[0]:
                break
            subtour.append(next_node)
            unvisited.remove(next_node)
            current_node = next_node
        subtours.append(subtour)
    return subtours

model = ConcreteModel()

model.x = Var(nodes, nodes, within=Binary)

model.obj = Objective(
    expr=sum(distances[i][j] * model.x[i, j] for i in nodes for j in nodes if distances[i][j] != -1),
    sense=maximize
)

model.out_edges = ConstraintList()
for i in nodes:
    model.out_edges.add(sum(model.x[i, j] for j in nodes if i != j) <= 1)

model.in_edges = ConstraintList()
for j in nodes:
    model.in_edges.add(sum(model.x[i, j] for i in nodes if i != j) <= 1)

model.same_edges = ConstraintList()
for i in nodes:
    model.same_edges.add(sum(model.x[i, j] for j in nodes if i != j)==sum(model.x[j, i] for j in nodes if i != j))

for i in nodes:
    for j in nodes:
        if distances[i][j] == -1:
            model.x[i, j].fix(0)

#closing the path
model.closed_path = Constraint(expr=sum(model.x[i, 0] for i in nodes if i != 0) == 1)

solver = SolverFactory('glpk')

# Solve the linear program until there is single colsed path
while True:
    result = solver.solve(model)
    x_values = {(i, j): model.x[i, j].value for i in nodes for j in nodes}
    subtours = check_for_subtours(x_values)
    
    if len(subtours) == 1:
        break
    
    #adding constraints
    for subtour in subtours:
        if len(subtour) > 1:
            model.in_edges.add(sum(model.x[i, j] for i in subtour for j in subtour if i != j) <= len(subtour) - 1)

# Printing the results
total_dist=0
print("Longest closed path:")
for i in nodes:
    for j in nodes:
        if value(model.x[i, j]) == 1:
            print(f"Node {i+1} to Node {j+1} (Distance: {distances[i][j]})")
            total_dist+=distances[i][j]

print("Total length of path = ",total_dist,"meters")
