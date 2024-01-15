#include <chrono>
#include "Node.h"
#include "AStarSearch.h"
#include "FocalSearch.h"

#define MAX 105

void printSolution(aStarSearch &starSearch, const Node &Start, const Node &Goal) {
	auto now = Goal;

	//print soln
	vector<Node> Path;
	while (starSearch.visited[now].parent_ != EOF) {
		Path.push_back(now);
		now = now.getNode(starSearch.visited[now].parent_);
	}
	Path.push_back(Start);
	reverse(Path.begin(), Path.end());
	for (auto &i : Path) cout << i;
}

void printSolutionFocal(FocalSearch &focal_search, const Node &Start, const Node &Goal) {
	auto now = Goal;

	//print soln
	vector<Node> Path;
	while (focal_search.visited[now].parent_ != EOF) {
		Path.push_back(now);
		now = now.getNode(focal_search.visited[now].parent_);
	}
	Path.push_back(Start);
	reverse(Path.begin(), Path.end());
	for (auto &i : Path) cout << i;
}

// for fs
void printFsSolution(FocalSearch &focal_search, const Node &Start, const Node &Goal) {
	auto now = Goal;

	//print soln
	vector<Node> Path;
	while (focal_search.visited[now].parent_ != EOF) {
		Path.push_back(now);
		now = now.getNode(focal_search.visited[now].parent_);
	}
	Path.push_back(Start);
	reverse(Path.begin(), Path.end());
	for (auto &i : Path) cout << i;
}

void executeSearch(const Node &Start, const Node &Goal, int heuristic, bool printSol = true) {

	// auto *starSearch = new aStarSearch();
	auto *focalSearch = new FocalSearch();

	// starSearch->setHeuristic(heuristic);
	focalSearch->setHeuristic(heuristic);

	auto startTime = chrono::steady_clock::now();
	// int nExpanded = starSearch->AStarSearch(Start, Goal);
	int nExpanded = focalSearch->FocalSearchAlgorithm(Start, Goal, 2, 0.2);
	auto endTime = chrono::steady_clock::now();

	auto diff = endTime - startTime;
	// cout << "No of Steps: " << (int) starSearch->visited[Goal].cost_ << endl;
	cout << "No of Steps: " << (int) focalSearch->visited[Goal].cost << endl;
	cout << "No of Nodes Expanded: " << nExpanded << endl;
	// cout << "No of Nodes Opened: " << starSearch->openedCount << endl;
	cout << "No of Nodes Opened: " << focalSearch->openedCount << endl;
	// cout << "No of Nodes Pushed: " << starSearch->nPushed << endl;
	cout << "No of Nodes Pushed: " << focalSearch->nPushed << endl;
	// cout << "Max Depth Reached: " << starSearch->max_depth << endl;
	cout << "Max Depth Reached: " << focalSearch->max_depth << endl;
	cout << "Execution Time: " << chrono::duration<double, milli>(diff).count() << "ms" << endl;
	cout << endl;
	fflush(stdout);

	// if (printSol) printSolution(*starSearch, Start, Goal);
	// delete starSearch;
	if (printSol) printSolutionFocal(*focalSearch, Start, Goal);
	delete focalSearch;
}

int main() {
	freopen(R"(C:\Users\DELL 7480\CLionProjects\npuzzle\in.txt)", "r", stdin);
	freopen(R"(C:\Users\DELL 7480\CLionProjects\npuzzle\out.txt)", "w", stdout);
	int boardSqSize = 3;
	cin >> boardSqSize;
	Node::boardSqSize = boardSqSize;

	Node Goal;
	for (int i = 0; i < boardSqSize; i++)
		for (int j = 0; j < boardSqSize; j++)
			Goal.A[i][j] = static_cast<puzzle_t>(i * Node::boardSqSize + j + 1);
	Goal.A[Node::boardSqSize - 1][Node::boardSqSize - 1] = 0;

	Node Start;
	int x;
	for (int i = 0; i < boardSqSize; i++)
		for (int j = 0; j < boardSqSize; j++)
			cin >> x, Start.A[i][j] = static_cast<puzzle_t>(x);

	cout << "Start: \n" << Start;
	cout << "Goal: \n" << Goal;
	// cout << aStarSearch::ManHattan(Start, Goal) << endl;
	// cout << aStarSearch::HammingDistance(Start, Goal) << endl;
	// cout << aStarSearch::nLinearConflicts(Start, Goal) << endl;

	if (!Start.isSolveAble()) {
		cout << "No Solution" << endl;
	} else {
		{
			cout << "# Linear Conflicts Heuristics: " << endl;
			executeSearch(Start, Goal, LINEAR_CONFLICT, false);

			cout << "# ManHattan Distance Heuristics: " << endl;
			executeSearch(Start, Goal, MANHATTAN_DISTANCE, false);

			cout << "#Hamming Distance Heuristics: " << endl;
			executeSearch(Start, Goal, HAMMING_DISTANCE, true);
		}
	}
}