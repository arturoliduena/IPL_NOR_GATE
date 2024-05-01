#include <iostream>
#include <vector>
#include <bitset>
#include <cmath>
#include <ilcplex/ilocplex.h>

using namespace std;

vector<int> Binary(int num, int base)
{
  vector<int> v;
  // Binary representation of num with base bits
  for (int i = base - 1; i >= 0; i--)
  {
    // Use bitwise AND operation to check if the i-th bit is set
    int bit = (num >> i) & 1;
    v.push_back(bit);
  }
  return v;
}

ILOSTLBEGIN
int main()
{
  int num_variables;
  cin >> num_variables;
  int rows = pow(2, num_variables);
  vector<pair<vector<int>, int>> truth_table;
  for (int i = 0; i < rows; i++)
  {
    int _temp;
    cin >> _temp;
    truth_table.push_back({Binary(i, num_variables), _temp});
  }

  int depth = 0;

  while (true)
  {
    int num_nodes = pow(2, depth + 1) - 1;
    int leave = pow(2, depth);
    IloEnv env;
    IloModel model(env);
    IloNumVarArray nodes(env, num_nodes * num_variables, 0, 1, ILOBOOL);
    IloNumVarArray is_NOR(env, num_nodes, 0, 1, ILOBOOL);

    IloExpr sum_nor(env);
    for (int i = 0; i < num_nodes; i++)
    {
      sum_nor += is_NOR[i];
    }
    model.add(IloMinimize(env, sum_nor));

    // Each node can have at most one variable assigned to it
    for (int i = 0; i < num_nodes; i++)
    {
      IloExpr sum(env);
      for (int j = 0; j < num_variables; j++)
      {
        sum += nodes[i * num_variables + j];
      }
      model.add(sum <= 1);

      // If is_NOR[i] is 1, then no variables are assigned to node i
      model.add(IloIfThen(env, is_NOR[i] == 1, sum == 0));

      int left_idx = i * 2 + 1;  // Left child: (current index * 2) + 1
      int right_idx = i * 2 + 2; // Right child: (current index * 2) + 2
      if (i > num_nodes - leave - 1)
      {
        // If the node is a leaf node cant't be a NOR node
        model.add(is_NOR[i] == 0);
      }
    }

    for (int idx = 0; idx < truth_table.size(); idx++)
    {
      IloNumVarArray tree(env, num_nodes, 0, 1, ILOBOOL);
      vector<int> xs = truth_table[idx].first;
      int y = truth_table[idx].second;
      model.add(tree[0] == y);
      for (int i = 0; i < num_nodes; i++)
      {
        IloExpr sum(env);
        for (int j = 0; j < num_variables; j++)
        {
          sum += nodes[i * num_variables + j] * xs[j];
        }
        int left_idx = i * 2 + 1;  // Left child: (current index * 2) + 1
        int right_idx = i * 2 + 2; // Right child: (current index * 2) + 2
        if (i < num_nodes - leave)
        {
          model.add(IloIfThen(env, is_NOR[i] == 1, tree[left_idx] <= 1 - tree[i]));
          model.add(IloIfThen(env, is_NOR[i] == 1, tree[right_idx] <= 1 - tree[i]));
          model.add(IloIfThen(env, is_NOR[i] == 1, (1 - tree[i]) <= (tree[left_idx] + tree[right_idx])));
          model.add(IloIfThen(env, is_NOR[i] == 1, -tree[i] <= 0));
        }
        model.add(IloIfThen(env, is_NOR[i] == 0, tree[i] == sum));
      }
    }

    IloCplex cplex(model);
    cplex.solve();

    // Check the solution status
    if (cplex.getStatus() == IloAlgorithm::Optimal || cplex.getStatus() == IloAlgorithm::Feasible)
    {
      IloNumArray vals(env);
      cplex.getValues(vals, nodes);
      std::cout << "num_nodes: " << num_nodes << std::endl;
      std::cout << "depth: " << depth << std::endl;
      std::cout << "num_variables: " << num_variables << std::endl;
      std::cout << "leave: " << leave << std::endl;
      std::cout << "______________nodes__________________" << std::endl;
      for (int i = 0; i < num_nodes; i++)
      {
        for (int j = 0; j < num_variables; j++)
        {
          std::cout << (int)vals[i * num_variables + j] << " ";
        }
        std::cout << " | ";
      }
      std::cout << std::endl;

      IloNumArray vals_types(env);
      cplex.getValues(vals_types, is_NOR);
      std::cout << "______________node-types__________________" << std::endl;
      for (int i = 0; i < num_nodes; i++)
      {
        std::cout << (int)vals_types[i] << " ";
      }
      std::cout << std::endl;
      env.end();
      break; // Found a solution, exit the loop
    }
    else
    {
      // CPLEX encountered an issue during solving
      std::cerr << "CPLEX failed to find a solution. Status: " << cplex.getStatus() << std::endl;
      env.end();
      depth++; // Increment depth if no feasible solution found
    }
  }
  return 0;
}
