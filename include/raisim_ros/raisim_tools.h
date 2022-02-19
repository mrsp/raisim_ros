#include <vector>
#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;


Eigen::VectorXd VecToEigen(std::vector<double> vec_)
{
  Eigen::VectorXd eigvec_(vec_.size());
  for(int i=0;i<vec_.size();i++)
  {
    eigvec_(i) = vec_[i];
  }
  return eigvec_;
}

std::vector<double> EigenToVec(Eigen::VectorXd  eigvec_)
{
  std::vector<double> vec_;
  vec_.resize(eigvec_.size());
  for(int i=0;i<eigvec_.size();i++)
  {
    vec_[i] = eigvec_(i);
  }
  return vec_;
}

Eigen::MatrixXd parseCSV(string filename, int data_dim)
{
  ifstream in(filename);
  vector<vector<double>> fields;

  if (in)
  {
    string line;

    while (getline(in, line))
    {
      stringstream sep(line);
      string field;

      fields.push_back(vector<double>());

      while (getline(sep, field, ','))
      {
        fields.back().push_back(stod(field));
      }
    }
  }
  cout << "Size " << fields[0].size() << std::endl;
  cout << "Data Size " << data_dim << std::endl;
  Eigen::MatrixXd data(fields.size(), data_dim);
  int i = 0;
  int j = 0;

  for (auto row : fields)
  {
    for (auto field : row)
    {
      //cout << field << ' ';
      data(i, j) = field;
      j++;
    }
    j = 0;
    i++;
    //cout<<"\n";
    //cout<<"\n";
  }
  return data;
}

Eigen::VectorXd PDControl(const Eigen::VectorXd &q, const Eigen::VectorXd &qd, const Eigen::VectorXd &dq, const Eigen::VectorXd &dqd, const Eigen::MatrixXd &P, const Eigen::MatrixXd &D)
{
  Eigen::VectorXd tau;
  tau.setZero(dq.size());
  tau = P*(qd-q) + D*(dqd-dq); 

  return tau;
}

Eigen::VectorXd PDControl(const Eigen::VectorXd &q, const Eigen::VectorXd &qd, const Eigen::VectorXd &dq, const Eigen::VectorXd &dqd,  Eigen::MatrixXd M,  Eigen::VectorXd h, const Eigen::MatrixXd &P, const Eigen::MatrixXd &D)
{
  Eigen::VectorXd tau;
  tau.setZero(dq.size());
  tau = M*(P*(qd-q) + D*(dqd-dq))+h;
  return tau;
}

void swapQuatWXYZ(Eigen::VectorXd &input_)
{
  Eigen::VectorXd tmp(input_.size());
  tmp = input_;
  input_[3] = tmp[6];
  input_[4] = tmp[3];
  input_[5] = tmp[4];
  input_[6] = tmp[5];
}
void swapQuatXYZW(Eigen::VectorXd &input_)
{
  Eigen::VectorXd tmp(input_.size());
  tmp = input_;
  input_[3] = tmp[4];
  input_[4] = tmp[5];
  input_[5] = tmp[6];
  input_[6] = tmp[3];
}
