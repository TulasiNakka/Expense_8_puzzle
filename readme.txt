                               /* Instructions for running Assignment1*/

1. Name and UTA ID: Tulasi Nakka, 1001928971
2. Programming language used and Version: Python 3.9.13


3. About Code: 
In my code, I have defined a class expense_8_puzzle where create global variables to run the algorithms. I have also written the code related to the dump file and tried to replicate it as given in the sample. I have implemented all the algorithms: bfs, ucs, dfs, dls, ids, greedy, and A* according to the pseudo codes. Checked running every algorithm and it is working.

4. Instructions to run:
To run the program, unzip the folder, navigate to the necessary path of the folder named "1001928971_assmt1" and give the following command in the terminal. [I have used VS Code for implementing and running the code]

command:
python expense_8_puzzle.py start.txt goal.txt <method> <dump-flag>

same as given in the assignment1 description below,

<start-file> and <goal-file> are required.

<method> can be
bfs - Breadth First Search
ucs - Uniform Cost Search
dfs - Depth First Search [Note: This part is EC for CSE 4308 students]
dls - Depth Limited Search (Note: Depth Limit will be obtained as a Console Input) [Note: This part is EC for CSE 4308 students]
ids - Iterative Deepening Search [Note: This part is EC for CSE 4308 students]
greedy - Greedy Seach
a* - A* Search (Note: if no <method> is given, this should be the default option)
If <dump-flag>  is given as true, search trace is dumped for analysis in trace-<date>-<time>.txt (Note: if <dump-flag> is not given, assume it is false)


5. These are instructions to run the code and I have also provided the required comments in the code.