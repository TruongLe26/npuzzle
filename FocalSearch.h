//
// Created by DELL 7480 on 11/21/2023.
// Paper: https://www.ijcai.org/Proceedings/2018/0199.pdf
//

#ifndef FOCALSEARCH_H
#define FOCALSEARCH_H

#include "Node.h"
#include <unordered_map>
#include <cmath>
#include <set>
#include <functional>
#include <vector>
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

	struct OpenSetComparator {
		const FocalSearch& fs;
		const Node& goal;

		OpenSetComparator(const FocalSearch& search, const Node& g) : fs(search), goal(g) {}

		bool operator()(const pair<pair<double, size_t>, Node>& a, const pair<pair<double, size_t>, Node>& b) const {
			auto it1 = fs.visited.find(a.second);
			auto it2 = fs.visited.find(b.second);
			const NodeInfo &aInfo = it1->second;
			const NodeInfo &bInfo = it2->second;
			// double f_a = fs.fValue(a.second, goal);
			// double f_b = fs.fValue(b.second, goal);
			
			// return make_pair(f_a, a.first.second) < make_pair(f_b, b.first.second);
			double fa = aInfo.cost;
			double fb = bInfo.cost;
			return make_pair(fa, a.first.second) < make_pair(fb, b.first.second);
		}
	};

	struct FocalSetComparator {
		const FocalSearch& fs;
		const Node& goal;

		FocalSetComparator(const FocalSearch& search, const Node& g) : fs(search), goal(g) {}

		bool operator()(const Node& a, const Node& b) const {
			return fs.hValue(a, goal) < fs.fValue(b, goal);
		}
	};

public:
	map<Node, NodeInfo> visited;
	size_t openedCount;
	int max_depth;
	int nPushed;
	int heuristicType = 0;

	int hValue(const Node& current, const Node& goal) const {
		if (heuristicType == MANHATTAN_DISTANCE) return ManHattan(current, goal);
		else if (heuristicType == HAMMING_DISTANCE) return HammingDistance(current, goal);
		else return LinearConflicts(current, goal);
		return 0;
	}

	int fValue(const Node& current, const Node& goal) const {
		// NodeInfo &currentInfo = visited[current];
		// if (heuristicType == MANHATTAN_DISTANCE) return currentInfo.cost + ManHattan(current, goal);
		// return 0;

		auto it = visited.find(current);
		if (it != visited.end()) {
			const NodeInfo &currentInfo = it->second;
			if (heuristicType == MANHATTAN_DISTANCE) {
				return currentInfo.cost + ManHattan(current, goal);
			} else if (heuristicType == HAMMING_DISTANCE) {
				return currentInfo.cost + HammingDistance(current, goal);
			} else {
				return currentInfo.cost + LinearConflicts(current, goal);
			}
		}
		return 0;
	}

    bool isValid(int x, int y) {
		return x >= 0 && y >= 0 && x < Node::boardSqSize && y < Node::boardSqSize;
	}

	static double HammingDistance(const Node &a, const Node &b) {
		int conflicts = 0;
		for (int i = 0; i < Node::boardSqSize; i++)
			for (int j = 0; j < Node::boardSqSize; j++)
				if (a.A[i][j] && a.A[i][j] != b.A[i][j]) conflicts++;
		return conflicts;
	}

	static double ManHattan(const Node &a, const Node &b) {
		int sum = 0;
		puzzle_t pR[Node::boardSqSize * Node::boardSqSize + 1];
		puzzle_t pC[Node::boardSqSize * Node::boardSqSize + 1];
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
		puzzle_t pR[Node::boardSqSize * Node::boardSqSize + 1];
		puzzle_t pC[Node::boardSqSize * Node::boardSqSize + 1];
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

	size_t hashNode(const Node &node) {
    	hash<puzzle_t> hasher;
    	size_t hashValue = 0;

    	for (int i = 0; i < Node::boardSqSize; ++i)
        	for (int j = 0; j < Node::boardSqSize; ++j)
            	hashValue ^= hasher(node.A[i][j]) + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);

    	return hashValue;
	}

	int FocalSearchAlgorithm(const Node &start, const Node &goal, double w, double ratio) {

		OpenSetComparator openSetComparator(*this, goal);
		set<pair<pair<double, size_t>, Node>, OpenSetComparator> open(openSetComparator);

		FocalSetComparator focalSetComparator(*this, goal);
		set<Node, FocalSetComparator> focalSet(focalSetComparator);

        map<Node, bool> isFocal;

        open.insert({{0, hashNode(start)}, start});
        focalSet.insert(start);

		visited[start] = {false, 0, EOF};

        int nExpanded = 0;
		max_depth = 0;
		nPushed = 0;

        while (!focalSet.empty()) {
        	// fmin is f(head(open))
			double fmin = open.empty() ? numeric_limits<double>::infinity() : open.begin()->first.first;
			// cout << "fmin is " << fmin << endl;

            Node current;

            // if (open.empty() || rand() % 100 < ratio * 100) {
            //     current = *focalSet.begin();
            //     focalSet.erase(focalSet.begin());
            // 	open.erase({{fValue(current, goal), hashNode(current)}, current});
            // } else {
            //     current = open.begin()->second;
            //     open.erase(open.begin());
            //     isFocal[current] = false;
            // }

        	current = *focalSet.begin();
        	focalSet.erase(focalSet.begin());
        	open.erase({{fValue(current, goal), hashNode(current)}, current});

			NodeInfo &curInfo = visited[current];
        	curInfo.isClosed = true;
            nExpanded++;
			//  cout << "Iteration: " << nExpanded << ", Focal Set Size: " << focalSet.size() << ", Open Set Size: " << open.size() << endl;
			// cout << current << endl;
        	max_depth = max(max_depth, visited[current].cost);

            if (current == goal) return nExpanded;
        	if (curInfo.cost > LIMIT_DEPTH) {
        		cout << "Height limit Exceeded @" << endl << current;
        		break;
        	}
        	if (visited.size() > NODE_LIMIT) {
        		cout << "Node limit Exceeded @" << endl << current;
        		break;
        	}

        	int zX = -1, zY = -1;
        	Node::getZeroPos(current, zX, zY);

            for (direction_t dir = 0; dir < 4; dir++) {
            	int zXnew = zX + dirX[dir];
            	int zYnew = zY + dirY[dir];

            	if (isValid(zXnew, zYnew)) {
            		Node v = current;
            		swap(v.A[zX][zY], v.A[zXnew][zYnew]);
            		bool isVisited = visited.find(v) != visited.end();
            		if (isVisited && visited[v].isClosed) continue;

            		double newCost = curInfo.cost + 1;
            		++nPushed;
            		visited[v] = {false, static_cast<cost_t>(newCost), Node::oppositeDirection(dir)};

					// cout << visited.size() << endl;

            		double Priority = newCost + Heuristic(v, goal);
            		open.insert({{Priority, hashNode(v)}, v});

            		if (Priority <= w * fmin) {
            			focalSet.insert(v);
            			isFocal[v] = true;
            		}
            	}
            }

            if (!open.empty() && open.begin()->first.first > fmin) {
                for (auto it = open.begin(); it != open.end(); ++it) {
                    if (fValue(it->second, goal) > w * fmin ||
                        fValue(it->second, goal) <= w * open.begin()->first.first) {
                        focalSet.insert(it->second);
                        isFocal[it->second] = true;
                    }
                }
            }

        }
		openedCount = visited.size();
        return nExpanded;
    }

	virtual ~FocalSearch() {
		heuristicType = 0;
		visited.clear();
	}
};

#endif //FOCALSEARCH_H
