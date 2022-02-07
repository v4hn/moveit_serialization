
#include <moveit_serialization/yaml-cpp/node_manipulation.h>

int main(int argc, char** argv) {
	YAML::Node n1;
	YAML::Node n2;

	YAML::merge_node(n1, n2);

	YAML::isSubset(n1, n2);

	YAML::scalar_compare<bool, int> cmp;
	cmp.equality(n1, n2);
}
