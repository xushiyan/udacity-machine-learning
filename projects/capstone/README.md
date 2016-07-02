# Project 5: Capstone
## Plot and Navigate a Virtual Maze

### Repository Structure
- [robot_motion_planning_report.pdf](robot_motion_planning_report.pdf) is the final report file.
- Folder [robot_motion_planning](robot_motion_planning/) contains all code of this project.
- Folder [res](res/) contains all resources used in the report.

### How to run code

1. Open a terminal with `python` command available.
2. `cd` to [robot_motion_planning](robot_motion_planning)
3. Run single test to see the output in terminal.
  ```
  python tester.py test_maze_01.txt
  ```

4. Run repeated tests to see the output appended in [test_maze_01.txt_results.txt](robot_motion_planning/test_maze_01.txt_results.txt)
  ```
  python tester_loop.py test_maze_01.txt
  ```

5. Modify possible values of robot's parameter `threshold` in range of `[0, 10)` at line 31 of [`tester_loop.py`](robot_motion_planning/tester_loop.py) to see different average scores appended in output files.

6. [`test_maze_01_var.txt`](robot_motion_planning/test_maze_01_var.txt) is the modified version of maze 01 used in free-form visualization.
