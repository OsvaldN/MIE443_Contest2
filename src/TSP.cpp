#include <TSP.h>

float eucDist(float x1, float y1, float x2, float y2){
    // returns euclidean distance between (x1,y1) and (x2,y2)
    return sqrt(pow((x2-x1), 2) + pow((y2-y1), 2) );
}


std::vector<std::vector<float>> distMatrix(std::vector<std::vector<float>> coords) {
    // Creates a NxN distance matrix from an (N,2) std::vector of coordinates
    int posCount = coords.size();
    std::vector<std::vector<float>> dM(posCount, std::vector<float> (posCount, 0));
    
    for (int i=0; i<posCount; i++){
        for (int j=0; j<posCount; j++){
            //TODO: use path planning distance here
            dM[i][j] = eucDist(
                coords[i][0], coords[i][1],
                coords[j][0], coords[j][1]
                );
        }
    }
    
    return dM;
}

std::tuple<std::vector<int>, float> greedy(std::vector<std::vector<float>> dM, int start){
    /*
    Greedy Solution to TSP
    dM: NxN std::vector of distances between N vertices
    start: vertex (as ordered in dM) to start greedy search at
     */
    std::vector<int> path{start};
    std::vector<bool> visited(dM.size(), false);
    visited[start] = true;
    
    float totalCost = 0;
    float greedyCost = std::numeric_limits<float>::infinity();
    int greedyStep;
    
    int loc = start;
    while (find(visited.begin(), visited.end(), false) != visited.end()){
        // reset greedyCost for the next step
        greedyCost = std::numeric_limits<float>::infinity();
        
        for (int i=0;i<dM.size();i++){
            if ( !(visited[i]) && (dM[loc][i] < greedyCost)){
                greedyStep = i;
                greedyCost = dM[loc][i];
            }
        }
        
        loc = greedyStep;
        path.push_back(loc);
        visited[loc] = true;
        totalCost += greedyCost;
    }
    
    //return home
    path.push_back(start);
    totalCost += dM[loc][start];
    
    return std::make_tuple(path, totalCost);
}

std::vector<int> bestGreedy(std::vector<std::vector<float>> dM, bool verbose){
    /*
    finds best greedy solution to TSP by solving greedy problem from each start node
    dM: NxN std::vector of distances between N vertices
     */
    float lowestCost = std::numeric_limits<float>::infinity();
    std::vector<int> bestPath;
    std::tuple<std::vector<int>, float> circTuple;
    
    for (int i=0;i<dM.size();i++){
        circTuple = greedy(dM, i);
        
        //TODO: import ROS_INFO and print using that
        if (verbose) {
            std::cout << "path starting at " << i << ": [";
            for (int j=0;j<std::get<0>(circTuple).size();j++){
                std::cout << std::get<0>(circTuple)[j] << " ";
            }
            std::cout << "] with cost:" << std::get<1>(circTuple) << std::endl;
        }
        
        if (std::get<1>(circTuple) < lowestCost){
            bestPath = std::get<0>(circTuple);
            lowestCost = std::get<1>(circTuple);
        }
    }
    
    return bestPath;
}
