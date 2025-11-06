
//   ==================================================
//   Smart Packet Routing System (BACKEND ENGINE)
 
//   1. BFS (Min Hops)
//   2. Dijkstra (Min Cost)
//   3. A* (Heuristic Min Cost)
//   4. DFS (First Path Found)
//   ==================================================
 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h> // For INT_MAX
#include <math.h>   // For A* heuristic 
#include <time.h>   // For simulation time
#include <float.h>  // For DBL_MAX
#include <stdbool.h> // For bool in DFS

#define MAX_ROUTERS 50
#define INFINITY INT_MAX


// 1. Graph & Network Structures

// A node in the adjacency list (represents a connection/edge)
typedef struct NeighborNode {
    int destination;        // Which router this edge connects to (index)
    int weight;             // Cost/distance/time for this edge
    struct NeighborNode* next;
} NeighborNode;

// The adjacency list for a *single* router
typedef struct AdjacencyList {
    NeighborNode* head; // Pointer to the first neighbor
} AdjacencyList;

// Structure to represent a Router (a node in the graph)
typedef struct Router {
    int id;
    char name[50];
    int x, y; // Coordinates for A* heuristic
} Router;

// The main Graph structure
typedef struct Graph {
    int numRouters;
    Router routers[MAX_ROUTERS];          // Array of all routers in the network
    AdjacencyList adjList[MAX_ROUTERS]; // Array of adjacency lists
} Graph;


// 2. Packet Structure (Conceptual)
typedef struct Packet {
    int id;
    int source;
    int destination;
    int priority;
    char data[100];
} Packet;


// 3. Queue (for BFS)
typedef struct QueueNode {
    int data;
    struct QueueNode* next;
} QueueNode;

typedef struct Queue {
    QueueNode *front, *rear;
} Queue;


// 4. Priority Queue (Min-Heap)
typedef struct PQNode {
    int vertex;
    double priority;
} PQNode;

typedef struct PriorityQueue {
    PQNode* nodes;
    int size;
    int capacity;
    int* pos;
} PriorityQueue;

// 5. Result Structure
typedef struct RoutingResult {
    char algorithmName[50];
    int path[MAX_ROUTERS];
    int pathLength;
    int cost;
    int hops;
    double timeMs;
    int success;
} RoutingResult;


// Function Prototypes
Graph* createGraph(int numRouters);
void addRouter(Graph* g, int id, const char* name, int x, int y);
void addEdge(Graph* g, int src, int dest, int weight);
void freeGraph(Graph* g);
Graph* setupNetwork();

Queue* createQueue();
void enqueue(Queue* q, int data);
int dequeue(Queue* q);
int isQueueEmpty(Queue* q);
void freeQueue(Queue* q);

PriorityQueue* createPriorityQueue(int capacity);
void swapPQNode(PQNode* a, PQNode* b);
void heapifyUp(PriorityQueue* pq, int idx);
void heapifyDown(PriorityQueue* pq, int idx);
int isPQEmpty(PriorityQueue* pq);
PQNode extractMin(PriorityQueue* pq);
void decreaseKey(PriorityQueue* pq, int vertex, double priority);
int isInPQ(PriorityQueue* pq, int vertex);
void freePriorityQueue(PriorityQueue* pq);

double simulateTime(int cost, int hops);
int reconstructPath(int parent[], int src, int dest, RoutingResult* result);

RoutingResult bfsRouting(Graph* g, int src, int dest);
RoutingResult dijkstraRouting(Graph* g, int src, int dest);
RoutingResult aStarRouting(Graph* g, int src, int dest);
double heuristic(Router r1, Router r2);

//  NEW DFS FUNCTIONS 
bool dfsUtil(Graph* g, int u, int dest, bool visited[], int parent[]);
RoutingResult dfsRouting(Graph* g, int src, int dest);


void printJsonSingleResult(Graph* g, RoutingResult res);
int compareResults(const void* a, const void* b);
void printJsonCompareResult(Graph* g, RoutingResult results[], int count);


// 1. Graph Utility Functions
Graph* createGraph(int numRouters) {
    Graph* g = (Graph*)malloc(sizeof(Graph));
    g->numRouters = numRouters;
    for (int i = 0; i < numRouters; i++) {
        g->adjList[i].head = NULL;
    }
    return g;
}

void addRouter(Graph* g, int id, const char* name, int x, int y) {
    if (id >= g->numRouters) return;
    g->routers[id].id = id;
    strcpy(g->routers[id].name, name);
    g->routers[id].x = x;
    g->routers[id].y = y;
}

void addEdge(Graph* g, int src, int dest, int weight) {
    NeighborNode* newNode = (NeighborNode*)malloc(sizeof(NeighborNode));
    newNode->destination = dest;
    newNode->weight = weight;
    newNode->next = g->adjList[src].head;
    g->adjList[src].head = newNode;

    newNode = (NeighborNode*)malloc(sizeof(NeighborNode));
    newNode->destination = src;
    newNode->weight = weight;
    newNode->next = g->adjList[dest].head;
    g->adjList[dest].head = newNode;
}

void freeGraph(Graph* g) {
    for (int i = 0; i < g->numRouters; i++) {
        NeighborNode* crawl = g->adjList[i].head;
        while (crawl != NULL) {
            NeighborNode* temp = crawl;
            crawl = crawl->next;
            free(temp);
        }
    }
    free(g);
}

Graph* setupNetwork() {
    int numRouters = 7;
    Graph* g = createGraph(numRouters);

    addRouter(g, 0, "A (Router)", 0, 0);
    addRouter(g, 1, "B (Server)", 2, 3);
    addRouter(g, 2, "C (Router)", 5, 1);
    addRouter(g, 3, "D (Switch)", 7, 4);
    addRouter(g, 4, "E (PC)", 10, 2);
    addRouter(g, 5, "F (Gateway)", 12, 5);
    addRouter(g, 6, "G (Firewall)", 4, 6);

    addEdge(g, 0, 1, 4);  // A - B (4)
    addEdge(g, 0, 2, 3);  // A - C (3)
    addEdge(g, 0, 6, 7);  // A - G (7)
    addEdge(g, 1, 3, 5);  // B - D (5)
    addEdge(g, 2, 3, 8);  // C - D (8)
    addEdge(g, 2, 4, 6);  // C - E (6)
    addEdge(g, 3, 5, 2);  // D - F (2)
    addEdge(g, 4, 5, 5);  // E - F (5)
    addEdge(g, 6, 3, 3);  // G - D (3)
    
    return g;
}

// 6. Queue Implementation
Queue* createQueue() {
    Queue* q = (Queue*)malloc(sizeof(Queue));
    q->front = q->rear = NULL;
    return q;
}

void enqueue(Queue* q, int data) {
    QueueNode* temp = (QueueNode*)malloc(sizeof(QueueNode));
    temp->data = data;
    temp->next = NULL;
    if (q->rear == NULL) {
        q->front = q->rear = temp;
        return;
    }
    q->rear->next = temp;
    q->rear = temp;
}

int dequeue(Queue* q) {
    if (q->front == NULL) return -1;
    QueueNode* temp = q->front;
    int data = temp->data;
    q->front = q->front->next;
    if (q->front == NULL) q->rear = NULL;
    free(temp);
    return data;
}

int isQueueEmpty(Queue* q) {
    return (q->front == NULL);
}

void freeQueue(Queue* q) {
    while (!isQueueEmpty(q)) {
        dequeue(q);
    }
    free(q);
}

// 7. Priority Queue (Min-Heap) Implementation
PriorityQueue* createPriorityQueue(int capacity) {
    PriorityQueue* pq = (PriorityQueue*)malloc(sizeof(PriorityQueue));
    pq->nodes = (PQNode*)malloc(capacity * sizeof(PQNode));
    pq->pos = (int*)malloc(capacity * sizeof(int));
    pq->size = 0;
    pq->capacity = capacity;
    for (int i = 0; i < capacity; i++) {
        pq->pos[i] = -1;
    }
    return pq;
}

void swapPQNode(PQNode* a, PQNode* b) {
    PQNode temp = *a;
    *a = *b;
    *b = temp;
}

void heapifyUp(PriorityQueue* pq, int idx) {
    int parent = (idx - 1) / 2;
    while (idx > 0 && pq->nodes[idx].priority < pq->nodes[parent].priority) {
        pq->pos[pq->nodes[idx].vertex] = parent;
        pq->pos[pq->nodes[parent].vertex] = idx;
        swapPQNode(&pq->nodes[idx], &pq->nodes[parent]);
        idx = parent;
        parent = (idx - 1) / 2;
    }
}

void heapifyDown(PriorityQueue* pq, int idx) {
    int smallest = idx;
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;

    if (left < pq->size && pq->nodes[left].priority < pq->nodes[smallest].priority)
        smallest = left;
    if (right < pq->size && pq->nodes[right].priority < pq->nodes[smallest].priority)
        smallest = right;

    if (smallest != idx) {
        int v1 = pq->nodes[idx].vertex;
        int v2 = pq->nodes[smallest].vertex;
        pq->pos[v1] = smallest;
        pq->pos[v2] = idx;
        swapPQNode(&pq->nodes[idx], &pq->nodes[smallest]);
        heapifyDown(pq, smallest);
    }
}

int isPQEmpty(PriorityQueue* pq) {
    return pq->size == 0;
}

PQNode extractMin(PriorityQueue* pq) {
    if (isPQEmpty(pq)) return (PQNode){-1, -1};
    PQNode root = pq->nodes[0];
    PQNode lastNode = pq->nodes[pq->size - 1];
    pq->nodes[0] = lastNode;
    pq->pos[root.vertex] = -1;
    pq->pos[lastNode.vertex] = 0;
    pq->size--;
    heapifyDown(pq, 0);
    return root;
}

void decreaseKey(PriorityQueue* pq, int vertex, double priority) {
    int i = pq->pos[vertex];
    if (i == -1) {
        i = pq->size;
        pq->size++;
    }
    pq->nodes[i].vertex = vertex;
    pq->nodes[i].priority = priority;
    pq->pos[vertex] = i;
    heapifyUp(pq, i);
}

int isInPQ(PriorityQueue* pq, int vertex) {
    return pq->pos[vertex] != -1;
}

void freePriorityQueue(PriorityQueue* pq) {
    free(pq->nodes);
    free(pq->pos);
    free(pq);
}


// 8. Result Helpers
double simulateTime(int cost, int hops) {
    double baseTime = (double)cost * 0.5 + (double)hops * 0.2;
    double jitter = (double)(rand() % 100) / 50.0;
    return baseTime + jitter;
}

int reconstructPath(int parent[], int src, int dest, RoutingResult* result) {
    int path[MAX_ROUTERS];
    int pathIndex = 0;
    int currentNode = dest;

    if (parent[dest] == -1 && src != dest) {
        result->success = 0;
        return 0;
    }

    while (currentNode != -1) {
        path[pathIndex++] = currentNode;
        currentNode = parent[currentNode];
    }

    result->pathLength = pathIndex;
    result->hops = pathIndex - 1;

    for (int i = 0; i < pathIndex; i++) {
        result->path[i] = path[pathIndex - 1 - i];
    }
    
    result->success = 1;
    return result->hops;
}

// 9. ALGORITHM 1: BFS Routing
RoutingResult bfsRouting(Graph* g, int src, int dest) {
    RoutingResult result;
    strcpy(result.algorithmName, "BFS (Min Hops)");
    result.success = 0;
    
    int dist[MAX_ROUTERS];
    int parent[MAX_ROUTERS];
    int visited[MAX_ROUTERS];

    for (int i = 0; i < g->numRouters; i++) {
        dist[i] = INFINITY;
        parent[i] = -1;
        visited[i] = 0;
    }

    Queue* q = createQueue();
    dist[src] = 0;
    visited[src] = 1;
    enqueue(q, src);

    while (!isQueueEmpty(q)) {
        int u = dequeue(q);
        if (u == dest) break;

        NeighborNode* temp = g->adjList[u].head;
        while (temp != NULL) {
            int v = temp->destination;
            if (!visited[v]) {
                visited[v] = 1;
                dist[v] = dist[u] + 1;
                parent[v] = u;
                enqueue(q, v);
            }
            temp = temp->next;
        }
    }

    int hops = reconstructPath(parent, src, dest, &result);
    if (result.success) {
        result.hops = hops;
        int totalCost = 0;
        for (int i = 0; i < result.pathLength - 1; i++) {
            int u = result.path[i];
            int v = result.path[i+1];
            NeighborNode* temp = g->adjList[u].head;
            while (temp) {
                if (temp->destination == v) {
                    totalCost += temp->weight;
                    break;
                }
                temp = temp->next;
            }
        }
        result.cost = totalCost;
        result.timeMs = simulateTime(result.cost, result.hops);
    } else {
        result.cost = -1;
        result.hops = -1;
        result.timeMs = -1;
    }

    freeQueue(q);
    return result;
}

// 10. ALGORITHM 2: Dijkstra Routing
RoutingResult dijkstraRouting(Graph* g, int src, int dest) {
    RoutingResult result;
    strcpy(result.algorithmName, "Dijkstra");
    result.success = 0;
    
    double dist[MAX_ROUTERS];
    int parent[MAX_ROUTERS];
    
    PriorityQueue* pq = createPriorityQueue(g->numRouters);

    for (int v = 0; v < g->numRouters; v++) {
        dist[v] = DBL_MAX;
        parent[v] = -1;
    }

    dist[src] = 0.0;
    decreaseKey(pq, src, dist[src]);

    while (!isPQEmpty(pq)) {
        PQNode uNode = extractMin(pq);
        int u = uNode.vertex;

        if (u == dest) break;

        NeighborNode* temp = g->adjList[u].head;
        while (temp != NULL) {
            int v = temp->destination;
            
            if (isInPQ(pq, v) || dist[v] == DBL_MAX) {
                double newDist = dist[u] + temp->weight;
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    parent[v] = u;
                    decreaseKey(pq, v, newDist);
                }
            }
            temp = temp->next;
        }
    }

    reconstructPath(parent, src, dest, &result);
    if (result.success) {
        result.cost = (int)dist[dest];
        result.timeMs = simulateTime(result.cost, result.hops);
    } else {
        result.cost = -1;
        result.hops = -1;
        result.timeMs = -1;
    }

    freePriorityQueue(pq);
    return result;
}

// 11. ALGORITHM 3: A* (A Star) Routing
double heuristic(Router r1, Router r2) {
    return sqrt(pow(r1.x - r2.x, 2) + pow(r1.y - r2.y, 2));
}

RoutingResult aStarRouting(Graph* g, int src, int dest) {
    RoutingResult result;
    strcpy(result.algorithmName, "A* Star");
    result.success = 0;
    
    int parent[MAX_ROUTERS];
    double gScore[MAX_ROUTERS];
    double fScore[MAX_ROUTERS];
    
    PriorityQueue* pq = createPriorityQueue(g->numRouters);

    for (int v = 0; v < g->numRouters; v++) {
        parent[v] = -1;
        gScore[v] = DBL_MAX;
        fScore[v] = DBL_MAX;
    }

    gScore[src] = 0.0;
    fScore[src] = heuristic(g->routers[src], g->routers[dest]);
    decreaseKey(pq, src, fScore[src]);

    while (!isPQEmpty(pq)) {
        PQNode uNode = extractMin(pq);
        int u = uNode.vertex;
        
        if (u == dest) {
            reconstructPath(parent, src, dest, &result);
            if (result.success) {
                result.cost = (int)gScore[dest];
                result.timeMs = simulateTime(result.cost, result.hops);
            }
            freePriorityQueue(pq);
            return result;
        }

        NeighborNode* temp = g->adjList[u].head;
        while (temp != NULL) {
            int v = temp->destination;
            double tentative_gScore = gScore[u] + temp->weight;
            
            if (tentative_gScore < gScore[v]) {
                parent[v] = u;
                gScore[v] = tentative_gScore;
                fScore[v] = gScore[v] + heuristic(g->routers[v], g->routers[dest]);
                decreaseKey(pq, v, fScore[v]);
            }
            temp = temp->next;
        }
    }

    result.success = 0;
    result.cost = -1;
    result.hops = -1;
    result.timeMs = -1;
    freePriorityQueue(pq);
    return result;
}

// 12. ALGORITHM 4: DFS Routing (NEW)
// (Finds the first path it can, not optimal)


bool dfsUtil(Graph* g, int u, int dest, bool visited[], int parent[]) {
    visited[u] = true;

    // Base case: If we found the destination
    if (u == dest) {
        return true;
    }

    // Recurse for all adjacent vertices
    NeighborNode* temp = g->adjList[u].head;
    while (temp != NULL) {
        int v = temp->destination;
        if (!visited[v]) {
            parent[v] = u;
            // If a path is found from this neighbor,
            // we stop and return true up the call stack.
            if (dfsUtil(g, v, dest, visited, parent)) {
                return true;
            }
        }
        temp = temp->next;
    }

    // If no path was found from any neighbor, we backtrack
    return false;
}

RoutingResult dfsRouting(Graph* g, int src, int dest) {
    RoutingResult result;
    strcpy(result.algorithmName, "DFS (First Path)");
    result.success = 0;

    bool visited[MAX_ROUTERS];
    int parent[MAX_ROUTERS];

    for (int i = 0; i < g->numRouters; i++) {
        visited[i] = false;
        parent[i] = -1;
    }

    // Start the recursive search
    bool pathFound = dfsUtil(g, src, dest, visited, parent);

    if (pathFound) {
        // Path was found, now reconstruct it and calculate cost
        int hops = reconstructPath(parent, src, dest, &result);
        if (result.success) {
            result.hops = hops;
            // Manually calculate the cost of the path DFS found
            int totalCost = 0;
            for (int i = 0; i < result.pathLength - 1; i++) {
                int u = result.path[i];
                int v = result.path[i+1];
                NeighborNode* temp = g->adjList[u].head;
                while (temp) {
                    if (temp->destination == v) {
                        totalCost += temp->weight;
                        break;
                    }
                    temp = temp->next;
                }
            }
            result.cost = totalCost;
            result.timeMs = simulateTime(result.cost, result.hops);
        }
    } else {
        // No path found
        result.success = 0;
        result.cost = -1;
        result.hops = -1;
        result.timeMs = -1;
    }

    return result;
}


// 13. Backend Output Functions

void printJsonSingleResult(Graph* g, RoutingResult res) {
    if (!res.success) {
        printf("{\"success\": false, \"name\": \"%s\"}", res.algorithmName);
        return;
    }

    printf("{\"success\": true, ");
    printf("\"name\": \"%s\", ", res.algorithmName);
    printf("\"cost\": %d, ", res.cost);
    printf("\"hops\": %d, ", res.hops);
    printf("\"timeMs\": %.2f, ", res.timeMs);
    printf("\"path\": [");
    for (int i = 0; i < res.pathLength; i++) {
        printf("{\"id\": %d, \"name\": \"%s\"}", 
            res.path[i], g->routers[res.path[i]].name);
        if (i < res.pathLength - 1) {
            printf(", ");
        }
    }
    printf("]}");
}

/**
 * Comparison function for qsort.
 */
int compareResults(const void* a, const void* b) {
    RoutingResult* resA = (RoutingResult*)a;
    RoutingResult* resB = (RoutingResult*)b;
    if (!resA->success && resB->success) return 1;
    if (resA->success && !resB->success) return -1;
    if (!resA->success && !resB->success) return 0;
    if (resA->cost < resA->cost) return -1;
    if (resA->cost > resB->cost) return 1;
    if (resA->timeMs < resB->timeMs) return -1;
    if (resA->timeMs > resB->timeMs) return 1;
    return 0;
}


/**
 * Prints the comparison results as a JSON array.
 */
void printJsonCompareResult(Graph* g, RoutingResult results[], int count) {
    qsort(results, count, sizeof(RoutingResult), compareResults);
    
    printf("["); // Start JSON array
    for (int i = 0; i < count; i++) {
        RoutingResult res = results[i];
        if (!res.success) {
            printf("{\"success\": false, \"name\": \"%s\"}", res.algorithmName);
        } else {
            printf("{\"success\": true, ");
            printf("\"name\": \"%s\", ", res.algorithmName);
            printf("\"cost\": %d, ", res.cost);
            printf("\"hops\": %d, ", res.hops);
            printf("\"timeMs\": %.2f, ", res.timeMs);
            
            if (i == 0) {
                 printf("\"performance\": \"⭐ Best\", ");
            } else if (res.cost == results[0].cost) {
                 printf("\"performance\": \"✅ Optimal\", ");
            } else if (strcmp(res.algorithmName, "BFS (Min Hops)") == 0) {
                 printf("\"performance\": \"⚠️ Min Hops\", ");
            } else {
                 printf("\"performance\": \"Suboptimal\", ");
            }

            printf("\"path\": [");
            for (int p = 0; p < res.pathLength; p++) {
                printf("{\"id\": %d, \"name\": \"%s\"}", 
                    res.path[p], g->routers[res.path[p]].name);
                if (p < res.pathLength - 1) {
                    printf(", ");
                }
            }
            printf("]}");
        }
        
        if (i < count - 1) {
            printf(", ");
        }
    }
    printf("]"); 
}

/**
 * NEW: main() function
 */
int main(int argc, char *argv[]) {
    srand(time(NULL));
    Graph* network = setupNetwork();

    if (argc != 4) {
        printf("{\"success\": false, \"error\": \"Invalid usage. Expected: ./routing_app <src> <dest> <algorithm>\"}");
        return 1; 
    }

    int src = atoi(argv[1]);
    int dest = atoi(argv[2]);
    char* algorithm = argv[3];

    if (src < 0 || src >= network->numRouters || dest < 0 || dest >= network->numRouters) {
        printf("{\"success\": false, \"error\": \"Invalid Router ID.\"}");
        freeGraph(network);
        return 1;
    }
    
    if (src == dest) {
         printf("{\"success\": false, \"error\": \"Source and Destination are the same.\"}");
        freeGraph(network);
        return 1;
    }

    // Run the chosen algorithm
    if (strcmp(algorithm, "dijkstra") == 0) {
        RoutingResult res = dijkstraRouting(network, src, dest);
        printJsonSingleResult(network, res);
    } 
    
    else if (strcmp(algorithm, "dfs") == 0) {
        RoutingResult res = dfsRouting(network, src, dest);
        printJsonSingleResult(network, res);
    } 
    
    else if (strcmp(algorithm, "bfs") == 0) {
        RoutingResult res = bfsRouting(network, src, dest);
        printJsonSingleResult(network, res);
    } else if (strcmp(algorithm, "astar") == 0) {
        RoutingResult res = aStarRouting(network, src, dest);
        printJsonSingleResult(network, res);
    } else if (strcmp(algorithm, "compare") == 0) {
        RoutingResult results[4];
        results[0] = dijkstraRouting(network, src, dest);
       
        results[1] = dfsRouting(network, src, dest); // Replaced Bellman-Ford
       
        results[2] = bfsRouting(network, src, dest);
        results[3] = aStarRouting(network, src, dest);
        printJsonCompareResult(network, results, 4);
    } else {
        
        printf("{\"success\": false, \"error\": \"Unknown algorithm. Use 'dijkstra', 'dfs', 'bfs', 'astar', or 'compare'.\"}");
        freeGraph(network);
        return 1;
    }
    
    printf("\n");
    freeGraph(network);
    return 0;
}