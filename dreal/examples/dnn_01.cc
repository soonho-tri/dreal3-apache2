#include <algorithm>
#include <numeric>
#include <vector>

#include "dreal/symbolic/symbolic.h"
#include "dreal/util/profiler.h"

namespace dreal {
using std::set;
using std::vector;

// element-wise vector sum
vector<Expression>* expr_vec_add(vector<Expression>* es1,
                                 vector<Expression>* es2) {
  vector<Expression>* out = new vector<Expression>(es1->size());
  std::transform(es1->begin(), es1->end(), es2->begin(), out->begin(),
                 std::plus<Expression>());
  return out;
}

// element-wise vector product
vector<Expression>* expr_vec_otimes(vector<Expression>* es1,
                                    vector<Expression>* es2) {
  vector<Expression>* out = new vector<Expression>(es1->size());
  std::transform(es1->begin(), es1->end(), es2->begin(), out->begin(),
                 std::multiplies<Expression>());
  return out;
}

// // Σ vᵢ
Expression Sum2(const vector<Expression>& expressions) {
  Expression z;
  for (const auto& e : expressions) {
    z += e;
  }
  return z;
}
}  // namespace dreal

int main() {
  const int N = 1000;
  const int N2 = 1000;
  std::vector<dreal::Variable> vars(N);
  std::vector<dreal::Expression> varscast(N);

  std::vector<dreal::Variable> vars2(N);
  std::vector<dreal::Expression> varscast2(N);
  for (int i = 0; i < N; ++i) {
    vars[i] = dreal::Variable{std::to_string(i)};
    varscast[i] = dreal::Expression{vars[i]};
  }

  for (int i = 0; i < N; ++i) {
    vars2[i] = dreal::Variable{std::to_string(i + N)};
    varscast2[i] = dreal::Expression{vars2[i]};
  }
  std::vector<dreal::Expression>* result = &varscast;

  std::vector<dreal::Expression>* out;
  std::vector<dreal::Expression>* out1;
  for (int i = 0; i < N2; ++i) {
    out = dreal::expr_vec_add(result, &varscast2);
    if (i != 0) {
      delete result;
    }
    out1 = dreal::expr_vec_otimes(out, &varscast);
    if (i != 0) {
      delete out;
    }
    result = out1;
    dreal::Expression exp = dreal::Sum2(*result);
    for (int j = 0; j < N; ++j) {
      std::cerr << i << " " << std::endl;
      (*result)[j] = exp + (*result)[j];
    }
    if (i % 1 == 0) std::cout << i << std::endl;
  }
}
