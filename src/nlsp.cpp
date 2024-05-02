#include <iostream>
#include <vector>
#include <bitset>
#include <cmath>
#include <ilcplex/ilocplex.h>

using namespace std;

void print(int num_variables, vector<pair<vector<int>, int>> truth_table, int depth, int num_nodes, IloNumArray is_NOR, IloNumArray nodes)
{
  cout << num_variables << endl;
  for (int idx = 0; idx < truth_table.size(); idx++)
  {
    int y = truth_table[idx].second;
    cout << y << endl;
  }
  int s = 0; // Number of NOR nodes in the circuit (size)
  for (int i = 0; i < num_nodes; i++)
  {
    if ((int)is_NOR[i] == 1)
    {
      s++;
    }
  }
  cout << depth << " " << s << endl;

  for (int i = 0; i < num_nodes; i++)
  {
    // <id> <code> <left> <right>
    int id = i + 1;

    if ((int)is_NOR[i] == 1)
    {
      int left_id = (i * 2 + 1) + 1;  // Left child: (current index * 2) + 1
      int right_id = (i * 2 + 2) + 1; // Right child: (current index * 2) + 2
      cout << id << " -1"
           << " " << left_id << " " << right_id << endl;
    }
    else
    {
      int elem = 0;
      // Parent: (current index - 1) // 2 (round down)
      int const parent_idx = (i - 1) / 2;
      if ((int)is_NOR[parent_idx] == 1)
      {
        for (int j = 0; j < num_variables; j++)
        {
          if ((int)nodes[i * num_variables + j] == 1)
          {
            elem = j + 1;
          }
        }
        cout << id << " " << elem << " 0 0 " << endl;
      }
    }
  }
}

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
    // sum_nor.end();

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
      IloIfThen if_NOR(env, is_NOR[i] == 1, sum == 0);
      model.add(if_NOR);
      sum.end();
      if_NOR.end();

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
      IloNumVarArray values(env, num_nodes, 0, 1, ILOBOOL);
      vector<int> xs = truth_table[idx].first;
      int y = truth_table[idx].second;
      model.add(tree[0] == y);
      for (int i = 0; i < num_nodes; i++)
      {
        IloExpr expr(env);
        for (int j = 0; j < num_variables; j++)
        {
          expr += nodes[i * num_variables + j] * xs[j];
        }
        // Define integer expression representing c1 * x_1 + c2 * x_2
        model.add(values[i] == expr);
        expr.end();
      }

      for (int i = 0; i < num_nodes; i++)
      {
        int left_idx = i * 2 + 1;  // Left child: (current index * 2) + 1
        int right_idx = i * 2 + 2; // Right child: (current index * 2) + 2
        if (i < num_nodes - leave)
        {
          model.add(IloIfThen(env, is_NOR[i] == 1, tree[i] == !(tree[left_idx] == 1 || tree[right_idx] == 1)));
          model.add(IloIfThen(env, is_NOR[i] == 0, tree[i] == values[i]));
        }
        else
        {
          model.add(tree[i] == values[i]);
        }
      }
    }

    IloCplex cplex(model);
    cplex.setOut(env.getNullStream());
    cplex.setError(env.getNullStream());
    // cplex.exportModel("model.lp");
    cplex.solve();

    // Check the solution status
    if (cplex.getStatus() == IloAlgorithm::Optimal || cplex.getStatus() == IloAlgorithm::Feasible)
    {
      IloNumArray vals(env);
      cplex.getValues(vals, nodes);

      IloNumArray vals_types(env);
      cplex.getValues(vals_types, is_NOR);
      print(num_variables, truth_table, depth, num_nodes, vals_types, vals);
      env.end();
      break; // Found a solution, exit the loop
    }
    else
    {
      env.end();
      depth++; // Increment depth if no feasible solution found
    }
  }
  return 0;
}