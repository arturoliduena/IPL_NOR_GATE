#include <iostream>
#include <vector>
#include <bitset>
#include <cmath>
#include <ilcplex/ilocplex.h>

using namespace std;

ILOSTLBEGIN
class Nlsp
{
protected:
  int num_variables;
  int depth;
  int num_nodes;
  vector<pair<vector<int>, int>> truth_table;

public:
  Nlsp(vector<pair<vector<int>, int>> truth_table, int num_variables, int depth) : num_variables(num_variables), depth(depth), truth_table(truth_table), num_nodes(pow(2, depth + 1) - 1)
  {
    IloEnv env;
    IloModel model(env);
    IloNumVarArray nodes(env, num_nodes * num_variables, 0, 1, ILOBOOL);
    IloNumVarArray is_NOR(env, num_nodes, 0, 1, ILOBOOL);

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
      IloIfThen constrain_nor_0(env, is_NOR[i] == 1, sum == 0);
      model.add(constrain_nor_0);

      // Parent: (current index - 1) // 2 (round down)
      int left_idx = i * 2 + 1;  // Left child: (current index * 2) + 1
      int right_idx = i * 2 + 2; // Right child: (current index * 2) + 2

      if (!(left_idx < num_nodes && right_idx < num_nodes))
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
        int left_idx = i * 2 + 1;  // Left child: (current index * 2) + 1
        int right_idx = i * 2 + 2; // Right child: (current index * 2) + 2
        if (!(left_idx < num_nodes && right_idx < num_nodes))
        {
          model.add(IloIfThen(env, is_NOR[i] == 1, tree[left_idx] <= 1 - tree[i]));
          model.add(IloIfThen(env, is_NOR[i] == 1, tree[right_idx] <= 1 - tree[i]));
          model.add(IloIfThen(env, is_NOR[i] == 1, 1 - tree[i] <= tree[right_idx] + tree[left_idx]));
          model.add(IloIfThen(env, is_NOR[i] == 1, -tree[i] <= 0));
        }
        IloExpr sum(env);
        for (int j = 0; j < num_variables; j++)
        {
          sum += nodes[i * num_variables + j] * xs[j];
        }
        model.add(IloIfThen(env, is_NOR[i] == 0, tree[i] == sum));
      }
    }

    IloCplex cplex(model);
    cplex.solve();
    IloNumArray vals(env);
    cplex.getValues(vals, nodes);
    std::cout << "______________nodes__________________" << std::endl;
    for (int i = 0; i < num_nodes; i++)
    {
      for (int j = 0; j < num_variables; j++)
      {
        std::cout << (int)vals[i * num_variables + j] << " ";
      }
      std::cout << std::endl;
    }

    IloNumArray vals(env);
    cplex.getValues(vals, is_NOR);
    std::cout << "______________node-types__________________" << std::endl;
    for (int i = 0; i < num_nodes; i++)
    {
      std::cout << (int)vals[i] << " ";
    }
    env.end();
  }
};

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

int main()
{
  int vars;
  cin >> vars;
  int rows = pow(2, vars);
  vector<pair<vector<int>, int>> truth_table;
  for (int i = 0; i < rows; i++)
  {
    int _temp;
    cin >> _temp;
    truth_table.push_back({Binary(i, vars), _temp});
  }

  int depth = 2;
  Nlsp *m = new Nlsp(truth_table, vars, depth);

  // while (true)
  // {
  //   Nlsp *m = new Nlsp(truth_table, vars, depth);
  //   if (m.solve())
  //   {
  //     m->print();
  //     delete m;
  //     break; // Found a solution, exit the loop
  //   }
  //   else
  //   {
  //     depth++; // Increment depth if no feasible solution found
  //   }
  // }
  return 0;
}