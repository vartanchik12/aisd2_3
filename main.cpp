#include<HashTable.h>


int main() {
    Graph<int, int> graph;

    graph.add_vertex(1);
    graph.add_vertex(2);
    graph.add_vertex(3);
    graph.add_vertex(4);
    graph.add_vertex(5);

    graph.add_edge(2, 1, 1);
    graph.add_edge(3, 1, 3);
    graph.add_edge(1, 4, 3);
    graph.add_edge(4, 3, 1);
    graph.add_edge(2, 3, 2);
    graph.add_edge(3, 5, 1);
    graph.add_edge(4, 5, 1);
    graph.add_edge(5, 2, 2);

    graph.print();
    cout << "\n";
    graph.remove_vertex(2);
    graph.remove_edge(4, 3);

    graph.print();
    cout << "\n";
    cout << "Walk Test" << endl;
    graph.walk(1);
    cout << "\nTest Dijkstra." << endl;
    std::vector<Graph<int, int>::Edge> path = graph.dijk(1, 3);
    auto sum = 0;
    std::cout << "Shortest path for from 1, to 3:" << std::endl;
    if (path.empty()) {
        std::cout << "No path found." << std::endl;
    }
    else {
        for (const auto& edge : path) {
            std::cout << "From: " << edge.from << " To: " << edge.to << " Distance: " << edge.distance << std::endl;
            sum += edge.distance;
        }
        cout << sum;
    }

    cout << "\nFind optimal location." << endl;
    int optimalWarehouse = graph.find_optimal_warehouse();

    cout << "Optimal warehouse location: " << optimalWarehouse << endl;

    return 0;
}