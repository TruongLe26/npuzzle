//
// Created by DELL 7480 on 11/21/2023.
//

#ifndef FOCALSEARCH_H
#define FOCALSEARCH_H

#include "Node.h"
#include <queue>
#include <vector>
#include <unordered_map>
#include <cmath>

#define OPEN 0
#define FOCAL 1

// more
#define MANHATTAN_DISTANCE 1
#define HAMMING_DISTANCE 2
#define LINEAR_CONFLICT 3

#define LIMIT_DEPTH 60
#define NODE_LIMIT 10000000

#define cost_ cost
#define parent_ parent

typedef int cost_t;
typedef int parent_t;

using namespace std;

class FocalSearch {
public:
	map<Node, NodeInfo> visited;
	size_t openedCount;
	int max_depth;
	int nPushed;

	int fValue(const Node& current, const Node& goal) const {
		if (heuristicType == MANHATTAN_DISTANCE) return ManHattan(current, goal);
		return 0;
	}

    bool isValid(int x, int y) { return x >= 0 && y >= 0 && x < Node::boardSqSize && y < Node::boardSqSize; }

    int heuristicType = 0;

	static double HammingDistance(const Node &a, const Node &b) {
		int conflicts = 0;
		for (int i = 0; i < Node::boardSqSize; i++)
			for (int j = 0; j < Node::boardSqSize; j++)
				if (a.A[i][j] && a.A[i][j] != b.A[i][j]) conflicts++;
		return conflicts;
	}

	static double ManHattan(const Node &a, const Node &b) {
		int sum = 0;
		puzzle_t pR[(Node::boardSqSize * Node::boardSqSize) + 1];
		puzzle_t pC[(Node::boardSqSize * Node::boardSqSize) + 1];
		for (int r = 0; r < Node::boardSqSize; r++) {
			for (int c = 0; c < Node::boardSqSize; c++) {
				pR[a.A[r][c]] = static_cast<puzzle_t>(r);
				pC[a.A[r][c]] = static_cast<puzzle_t>(c);
			}
		}
		for (int r = 0; r < Node::boardSqSize; r++)
			for (int c = 0; c < Node::boardSqSize; c++)
				if (b.A[r][c])
					sum += abs(pR[b.A[r][c]] - r) + abs(pC[b.A[r][c]] - c);
		return sum;
	}

	static double nLinearConflicts(const Node &a, const Node &b) {
		int conflicts = 0;
		puzzle_t pR[(Node::boardSqSize * Node::boardSqSize) + 1];
		puzzle_t pC[(Node::boardSqSize * Node::boardSqSize) + 1];
		for (int r = 0; r < Node::boardSqSize; r++) {
			for (int c = 0; c < Node::boardSqSize; c++) {
				pR[a.A[r][c]] = static_cast<puzzle_t>(r);
				pC[a.A[r][c]] = static_cast<puzzle_t>(c);
			}
		}

		for (int r = 0; r < Node::boardSqSize; r++) {
			for (int cl = 0; cl < Node::boardSqSize; cl++) {
				for (int cr = cl + 1; cr < Node::boardSqSize; cr++) {
					if (b.A[r][cl] && b.A[r][cr] && r == pR[b.A[r][cl]] && pR[b.A[r][cl]] == pR[b.A[r][cr]] &&
					    pC[b.A[r][cl]] > pC[b.A[r][cr]]) {
							conflicts++;
					}
				}
			}
		}

		for (int c = 0; c < Node::boardSqSize; c++) {
			for (int rU = 0; rU < Node::boardSqSize; rU++) {
				for (int rD = rU + 1; rD < Node::boardSqSize; rD++) {
					if (b.A[rU][c] && b.A[rD][c] && c == pC[b.A[rU][c]] && pC[b.A[rU][c]] == pC[b.A[rD][c]] &&
					    pR[b.A[rU][c]] > pR[b.A[rD][c]]) {
							conflicts++;
					}
				}
			}
		}

		return conflicts;
	}

	static double LinearConflicts(const Node &a, const Node &b) {
		return ManHattan(a, b) + 2 * nLinearConflicts(a, b);
	}

	double Heuristic(const Node &a, const Node &b) const {
		if (heuristicType == HAMMING_DISTANCE) return HammingDistance(a, b);
		if (heuristicType == MANHATTAN_DISTANCE) return ManHattan(a, b);
		if (heuristicType == LINEAR_CONFLICT) return LinearConflicts(a, b);
		return 0;
	}

	void setHeuristic(int heuristic = MANHATTAN_DISTANCE) {
		heuristicType = heuristic;
	}

	int FocalSearchAlgorithm(const Node &start, const Node &goal, double w, double ratio) {
        priority_queue<pair<double, Node>> open;
        priority_queue<Node, vector<Node>, greater<Node>> focal;
        map<Node, bool> isOpen;
        map<Node, bool> isFocal;

        open.push({0, start});
        focal.push(start);
        isOpen[start] = true;
        int nExpanded = 0;

        while (!focal.empty()) {
            Node current;
            if (open.empty() || ((rand() % 100) < (ratio * 100))) {
                current = focal.top();
                focal.pop();
                isFocal[current] = false;
            } else {
                current = open.top().second;
                open.pop();
                isOpen[current] = false;
            }

			// NodeInfo &curInfo = visited[current];
            nExpanded++;

            if (current == goal) return nExpanded;

            for (direction_t dir = 0; dir < 4; dir++) {
                int zX, zY;

                if (Node::getZeroPos(current, zX, zY)) {
                    int zXnew = zX + dirX[dir];
                    int zYnew = zY + dirY[dir];

                    if (isValid(zXnew, zYnew)) {
                        Node v = current.getNode(dir, zX, zY);
                        // double newCost = v.Heuristic(goal);
						// double newCost = curInfo.cost + 1;
                    	double newCost = fValue(v, goal);

                        if (!isOpen[v] && !isFocal[v]) {
                            open.push({newCost, v});
                            isOpen[v] = true;
                        } else if (newCost < w * fValue(current, goal)) {
                            focal.push(v);
                            isFocal[v] = true;
                        }
                    }
                }
            }
        	// Update lower bound
            if (!open.empty() && open.top().first < w * fValue(current, goal)) {
                for (auto it = isOpen.begin(); it != isOpen.end(); ++it) {
                    if (fValue(it->first, goal) > w * fValue(current, goal) ||
                        fValue(it->first, goal) < w * open.top().first) {
                        focal.push(it->first);
                        isFocal[it->first] = true;
                    }
                }
            }
        }

        return nExpanded;
    }

	virtual ~FocalSearch() {
		heuristicType = 0;
		visited.clear();
	}
};

#endif //FOCALSEARCH_H
