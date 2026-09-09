#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <queue>
#include <limits>
#include <algorithm>

struct Node {
    cv::Point position;  // 格子坐标 (x, y)
    int g;               // 从起点到这里的累计代价
    int h;               // 到终点的估计剩余代价

    int f() const {
        return g + h;
    }
};

int heuristic(const cv::Point& position, const cv::Point& goal)
{
    return std::abs(position.x - goal.x)
         + std::abs(position.y - goal.y);
}

struct CompareNode {
    bool operator()(const Node& a, const Node& b) const
    {
        return a.f() > b.f();
    }
};



int main()
{
    const int rows = 20;       // 地图有多少行
    const int cols = 30;       // 地图有多少列
    const int cell_size = 25;  // 每个格子的显示边长，单位为像素
    // 地图数据：0 表示可通行，1 表示障碍物。
    std::vector<std::vector<int>> grid(rows, std::vector<int>(cols, 0));

    // 设置一个障碍物。
    grid[5][10] = 1;
    grid[7][3] = 1;
    grid[0][1] = 1;

    // 第 10 列形成竖墙，只在最下面一行留下通道。
    // 这里的 10 是从 0 开始的列下标。
    for (int row = 0; row < rows; ++row) {
        grid[row][10] = 1;
    }

    cv::Mat canvas(
        rows * cell_size,
        cols * cell_size,
        CV_8UC3,
        cv::Scalar(255, 255, 255)
    );

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (grid[row][col] == 1) {
                cv::rectangle(
                    canvas,
                    cv::Rect(
                        col * cell_size,
                        row * cell_size,
                        cell_size,
                        cell_size
                    ),
                    cv::Scalar(0, 0, 0),
                    cv::FILLED
                );
            }
        }
    }

    // 绘制横向网格线。
    for (int row = 0; row <= rows; ++row) {
        const int y = (row == rows)
            ? canvas.rows - 1
            : row * cell_size;

        cv::line(canvas,
                 cv::Point(0, y),
                 cv::Point(canvas.cols - 1, y),
                 cv::Scalar(200, 200, 200));
    }

    // 绘制纵向网格线。
    for (int col = 0; col <= cols; ++col) {
        const int x = (col == cols)
            ? canvas.cols - 1
            : col * cell_size;

        cv::line(canvas,
                 cv::Point(x, 0),
                 cv::Point(x, canvas.rows - 1),
                 cv::Scalar(200, 200, 200));
    }
    // Point 使用 (x, y)，在栅格地图中对应 (列, 行)。
    const cv::Point start(0, 0);
    const cv::Point goal(25, 15);

    // 起点：绿色圆点，放在格子中心。
    cv::circle(
        canvas,
        cv::Point(
            start.x * cell_size + cell_size / 2,
            start.y * cell_size + cell_size / 2
        ),
        cell_size / 3,
        cv::Scalar(0, 180, 0),
        cv::FILLED
    );

    // 终点：红色圆点。
    cv::circle(
        canvas,
        cv::Point(
            goal.x * cell_size + cell_size / 2,
            goal.y * cell_size + cell_size / 2
        ),
        cell_size / 3,
        cv::Scalar(0, 0, 255),
        cv::FILLED
    );

    // (列增量, 行增量)，图像坐标中 y 向下增大。
    const std::vector<cv::Point> motions = {
        {0, 1},   // 下
        {1, 0},   // 右
        {0, -1},  // 上
        {-1, 0}   // 左
    };

    const cv::Point test_position(0, 1);
    Node test_node{
        test_position,
        1,
        heuristic(test_position, goal)
    };

        // 尚未到达的格子，代价设为一个很大的值。
    const int INF = std::numeric_limits<int>::max();

    std::vector<std::vector<int>> g_score(
        rows, std::vector<int>(cols, INF)
    );

    // (-1, -1) 表示还没有记录父节点。
    std::vector<std::vector<cv::Point>> parent(
        rows,
        std::vector<cv::Point>(cols, cv::Point(-1, -1))
    );

    // 起点到自身的代价为 0。
    g_score[start.y][start.x] = 0;

    // 待搜索队列：初始时只放入起点。
    std::priority_queue<Node, std::vector<Node>, CompareNode> open_list;

    open_list.push(Node{
        start,
        0,
        heuristic(start, goal)
    });

    // 初始化时已放入起点，因此此处队列非空。
    bool found = false;

    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();

        // 如果后来发现了更短路线，跳过队列中残留的旧记录。
        if (current.g !=
            g_score[current.position.y][current.position.x]) {
            continue;
        }

        // 当终点被取出时，结束搜索。
        if (current.position == goal) {
            found = true;
            std::cout << "Goal found, cost=" << current.g << '\n';
            break;
        }

        for (const auto& motion : motions) {
            cv::Point next = current.position + motion;

            if (next.x < 0 || next.x >= cols ||
                next.y < 0 || next.y >= rows) {
                continue;
            }

            if (grid[next.y][next.x] == 1) {
                continue;
            }

            const int new_g = current.g + 1;

            if (new_g < g_score[next.y][next.x]) {
                g_score[next.y][next.x] = new_g;

                parent[next.y][next.x] = current.position;

                open_list.push(Node{
                    next,
                    new_g,
                    heuristic(next, goal)
                });
            }
        }
    }

    if (!found) {
        std::cout << "No path found\n";
    }

    std::vector<cv::Point> path;

    if (found) {
        cv::Point p = goal;

        // 从终点沿父节点回到起点。
        while (p != start) {
            path.push_back(p);
            p = parent[p.y][p.x];
        }
        path.push_back(start);

        // 当前顺序为终点到起点，反转为起点到终点。
        std::reverse(path.begin(), path.end());

        std::cout << "Path points: " << path.size()
                << ", steps: " << path.size() - 1 << '\n';

        // 将相邻路径格子的中心连起来。
        for (std::size_t i = 1; i < path.size(); ++i) {
            const cv::Point from(
                path[i - 1].x * cell_size + cell_size / 2,
                path[i - 1].y * cell_size + cell_size / 2
            );

            const cv::Point to(
                path[i].x * cell_size + cell_size / 2,
                path[i].y * cell_size + cell_size / 2
            );

            cv::line(
                canvas, from, to,
                cv::Scalar(255, 0, 0), 2
            );
        }
    }

    cv::imshow("A* Grid Map", canvas);
    cv::waitKey(0);

    return 0;
}