// JSON TOOLS for parse json
#ifndef JSON_TOOLS
#define JSON_TOOLS
#include <nlohmann/json.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>

// Eigen
#include <Eigen/Eigen>

// for convenience
using json = nlohmann::json;


void matrix_parse(Eigen::Matrix4d& matrix, const json& json_matrix) {
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            matrix(i, j) = json_matrix[i][j];
        }
    }
}

inline bool exists_file(const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

#endif