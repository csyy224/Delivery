#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <sstream>

namespace Utils
{
	inline constexpr double INF = 1e18;
    inline constexpr double EPS = 1e-9;

    inline std::vector<std::string> split(const std::string& s, char delimiter)
	{
		std::vector<std::string> tokens;
		std::string token;
		std::istringstream tokenStream(s);
		while(std::getline(tokenStream,token,delimiter)) tokens.push_back(token);
		return tokens;
	}

    inline bool doubleEquals(double a, double b, double eps = EPS) { return std::abs(a - b) < eps; }
};
#endif