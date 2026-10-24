#include <iostream>
#include <queue>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
using namespace std;

const int MAX = 100;           
const int INF = 1000000000;     

struct Neighbor {
    int city;
    int cost;
    Neighbor* next;
    Neighbor(int c, int w) : city(c), cost(w), next(nullptr) {}
};

class Graph {
private:
    string cities[MAX];
    Neighbor* head[MAX];
    int cityCount;

    int getCityIndex(const string& name) {
        for (int i = 0; i < cityCount; i++)
            if (cities[i] == name) return i;
        if (cityCount == MAX) return -1;
        cities[cityCount] = name;
        head[cityCount] = nullptr;
        return cityCount++;
    }
    void addNeighbor(int from, int to, int cost) {
        Neighbor* n = new Neighbor(to, cost);
        n->next = head[from];
        head[from] = n;
    }

public:
    Graph() : cityCount(0) {
        for (int i = 0; i < MAX; i++) head[i] = nullptr;
    }
    Graph(const Graph&) = delete;     //   dosent allow coping of graph object   ,so their isnt one tha one object that points to the same memory location    
    Graph& operator=(const Graph&) = delete; 

    void addRoad(const string& city1, const string& city2, int cost) {   
        int a = getCityIndex(city1);
        int b = getCityIndex(city2);
        if (a == -1 || b == -1) { cerr << "Too many cities\n"; return; }
        addNeighbor(a, b, cost);// undirected graph, so add both directions
        addNeighbor(b, a, cost);
    }

    bool loadGraph(const string& filename) {
        ifstream file(filename.c_str());
        if (!file.is_open()) { cerr << "Unable to open file\n"; return false; }
        string line;
        while (getline(file, line)) {
            if (line.find("END OF INPUT") != string::npos) break;
            stringstream row(line);
            string city1, city2; int cost;
            if (row >> city1 >> city2 >> cost) addRoad(city1, city2, cost);
        }
        return true;
    }

    int findCityIndex(const string& name) const {
        for (int i = 0; i < cityCount; i++)
            if (cities[i] == name) return i;
        return -1;
    }
    const string& getCityName(int index) const { return cities[index]; }
    const Neighbor* getNeighbors(int index) const { return head[index]; }
    int getCityCount() const { return cityCount; }

    int getRoadCost(int a, int b) const {
        for (const Neighbor* n = head[a]; n; n = n->next)
            if (n->city == b) return n->cost;
        return 0;
    }

    ~Graph() {
        for (int i = 0; i < cityCount; i++) {
            Neighbor* current = head[i];
            while (current) { Neighbor* t = current; current = current->next; delete t; }
        }
    }
};


struct SearchResult {
    bool found = false;
    int path[MAX];            
    int pathLength = 0;
    int totalDistance = 0;
    int nodesGenerated = 0;
    int nodesExpanded = 0;
    int maxFringeSize = 0;
};


class SearchAlgorithm {
public:
    virtual ~SearchAlgorithm() {}
    virtual SearchResult search(const Graph& graph, int start, int goal) = 0;

protected:
    void buildPath(const Graph& graph, const int parent[], int start, int goal, SearchResult& r) const {
        int reversed[MAX], len = 0;
        for (int c = goal; c != -1; c = parent[c]) reversed[len++] = c;
        r.pathLength = len;
        for (int i = 0; i < len; i++) r.path[i] = reversed[len - 1 - i];
        computeDistance(graph, r);
        r.found = true;
        (void)start;
    }
    void computeDistance(const Graph& graph, SearchResult& r) const {
        r.totalDistance = 0;
        for (int i = 1; i < r.pathLength; i++)
            r.totalDistance += graph.getRoadCost(r.path[i - 1], r.path[i]);
    }
};


class BFS : public SearchAlgorithm {
public:
    SearchResult search(const Graph& graph, int start, int goal) override {
        SearchResult r;
        bool visited[MAX] = {false};
        int parent[MAX];
        for (int i = 0; i < MAX; i++) parent[i] = -1;

        queue<int> q;
        q.push(start); visited[start] = true; r.nodesGenerated = 1;

        while (!q.empty()) {
            if ((int)q.size() > r.maxFringeSize) r.maxFringeSize = q.size();
            int cur = q.front(); q.pop();
            r.nodesExpanded++;
            if (cur == goal) { buildPath(graph, parent, start, goal, r); return r; }
            for (const Neighbor* n = graph.getNeighbors(cur); n; n = n->next) {
                if (visited[n->city]) continue;
                visited[n->city] = true;
                parent[n->city] = cur;
                q.push(n->city);
                r.nodesGenerated++;
            }
        }
        return r;
    }
};


class DFS : public SearchAlgorithm {
public:
    SearchResult search(const Graph& graph, int start, int goal) override {
        SearchResult r;
        const int CAP = MAX * MAX;
        static int stackCity[CAP], stackParent[CAP];
        int top = 0;
        bool visited[MAX] = {false};
        int parent[MAX];
        for (int i = 0; i < MAX; i++) parent[i] = -1;

        stackCity[top] = start; stackParent[top] = -1; top++;
        r.nodesGenerated = 1;

        while (top > 0) {
            if (top > r.maxFringeSize) r.maxFringeSize = top;
            top--;
            int cur = stackCity[top], from = stackParent[top];
            if (visited[cur]) continue;
            visited[cur] = true;
            parent[cur] = from;
            r.nodesExpanded++;
            if (cur == goal) { buildPath(graph, parent, start, goal, r); return r; }
            for (const Neighbor* n = graph.getNeighbors(cur); n; n = n->next) {
                if (visited[n->city] || top >= CAP) continue;
                stackCity[top] = n->city; stackParent[top] = cur; top++;
                r.nodesGenerated++;
            }
        }
        return r;
    }
};


class IterativeDeepening : public SearchAlgorithm {
public:
    SearchResult search(const Graph& graph, int start, int goal) override {
        SearchResult r;
        for (int limit = 0; limit <= graph.getCityCount(); limit++) {
            bool onPath[MAX] = {false};
            int pathStack[MAX];
            bool cutoff = false;
            r.nodesGenerated++;                        
            if (depthLimited(graph, start, goal, 0, limit, pathStack, onPath, r, cutoff)) {
                r.found = true;
                computeDistance(graph, r);
                return r;
            }
            if (!cutoff) break;     
        }
        return r;
    }

private:
    bool depthLimited(const Graph& graph, int city, int goal, int depth, int limit,
                      int pathStack[], bool onPath[], SearchResult& r, bool& cutoff) const {
        r.nodesExpanded++;
        if (depth + 1 > r.maxFringeSize) r.maxFringeSize = depth + 1;
        pathStack[depth] = city;

        if (city == goal) {
            r.pathLength = depth + 1;
            for (int i = 0; i <= depth; i++) r.path[i] = pathStack[i];
            return true;
        }
        if (depth == limit) {
            if (graph.getNeighbors(city)) cutoff = true;
            return false;
        }
        onPath[city] = true;
        for (const Neighbor* n = graph.getNeighbors(city); n; n = n->next) {
            if (onPath[n->city]) continue;             // avoid cycles
            r.nodesGenerated++;
            if (depthLimited(graph, n->city, goal, depth + 1, limit, pathStack, onPath, r, cutoff))
                return true;
        }
        onPath[city] = false;
        return false;
    }
};


class UCS : public SearchAlgorithm {
public:
    SearchResult search(const Graph& graph, int start, int goal) override {
        SearchResult r;
        int dist[MAX], parent[MAX];
        bool closed[MAX] = {false};
        for (int i = 0; i < MAX; i++) { dist[i] = INF; parent[i] = -1; }
        dist[start] = 0; r.nodesGenerated = 1;

        while (true) {
            int cur = -1, fringeSize = 0;
            for (int i = 0; i < graph.getCityCount(); i++) {
                if (closed[i] || dist[i] == INF) continue;
                fringeSize++;
                if (cur == -1 || dist[i] < dist[cur]) cur = i;
            }
            if (cur == -1) return r;                   // fringe empty: no route
            if (fringeSize > r.maxFringeSize) r.maxFringeSize = fringeSize;

            closed[cur] = true;
            r.nodesExpanded++;
            if (cur == goal) { buildPath(graph, parent, start, goal, r); return r; }

            for (const Neighbor* n = graph.getNeighbors(cur); n; n = n->next) {
                if (closed[n->city]) continue;
                if (dist[cur] + n->cost < dist[n->city]) {
                    dist[n->city] = dist[cur] + n->cost;
                    parent[n->city] = cur;
                    r.nodesGenerated++;
                }
            }
        }
    }
};


void printResult(const Graph& graph, const SearchResult& r) {
    if (!r.found) { cout << "distance: infinity\nroute:\nnone\n"; return; }
    cout << "distance: " << r.totalDistance << " km\nroute:\n";
    if (r.pathLength == 1) { cout << "none\n"; return; }
    for (int i = 1; i < r.pathLength; i++)
        cout << graph.getCityName(r.path[i - 1]) << " to " << graph.getCityName(r.path[i])
             << ", " << graph.getRoadCost(r.path[i - 1], r.path[i]) << " km\n";
}

int main(int argc, char* argv[]) {
    if (argc != 5) {
        cerr << "Usage: find_route algorithm input_filename origin_city destination_city\n";
        return 1;
    }
    string algo = argv[1], filename = argv[2], origin = argv[3], destination = argv[4];

    Graph graph;
    if (!graph.loadGraph(filename)) return 1;

    SearchAlgorithm* searcher = nullptr;
    if (algo == "BFS") searcher = new BFS();
    else if (algo == "DFS") searcher = new DFS();
    else if (algo == "IDS" || algo == "IDT") searcher = new IterativeDeepening();
    else if (algo == "UCS") searcher = new UCS();
    else { cerr << "Unknown algorithm: " << algo << "\n"; return 1; }

    int s = graph.findCityIndex(origin), t = graph.findCityIndex(destination);
    SearchResult result;
    double ms = 0;
    if (s != -1 && t != -1) {
        auto t0 = chrono::high_resolution_clock::now();
        result = searcher->search(graph, s, t);
        auto t1 = chrono::high_resolution_clock::now();
        ms = chrono::duration<double, milli>(t1 - t0).count();
    }
    printResult(graph, result);
    cerr << "[generated: " << result.nodesGenerated << ", expanded: " << result.nodesExpanded
         << ", max fringe: " << result.maxFringeSize << ", time: " << ms << " ms]\n";
    delete searcher;
    return 0;
}