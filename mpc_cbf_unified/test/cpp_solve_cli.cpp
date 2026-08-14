// Copyright (c) 2026, Ali-Eimaan. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Test-only executable: one JSON object per line on stdin, one JSON object per
// line on stdout, one solve per line — the C++/Python parity harness for
// A7. It is located by
// test_recursive_feasibility.py through the MPC_CBF_CPP_SOLVE_CLI environment
// variable set in CMakeLists.txt, with a cwd-relative fallback.
//
//   stdin :  {"x0": [...], "obstacles": [{"position": [...], "velocity": [...],
//                                         "radius": r, "is_dynamic": false}, ...],
//             "x_ref": [...]}
//   stdout:  {"status": "SUCCESS", "u0": [...], "solve_time_ms": 1.23,
//             "cbf_values": [...], "first_active_cbf_step": -1,
//             "first_active_obstacle": -1, "infeasibility_reason": ""}
//
// The scenario (double integrator, N = 8, dt = 0.1, gamma = 0.3, inflation
// 0.2) is fixed to match the fixture in test_recursive_feasibility.py and
// test_mpc_cbf_feasibility.cpp exactly, so the two front-ends solve the same
// problem.

#include <mpc_cbf_unified/mpc_cbf_solver.hpp>

#include <Eigen/Dense>

#include <fcntl.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace mpc_cbf_unified;

namespace
{

// ---------------------------------------------------------------------------
// Minimal JSON — enough for the fixed schema above, header-only, no deps.
// ---------------------------------------------------------------------------

struct Json
{
  enum class Type
  {
    kNull,
    kBool,
    kNumber,
    kString,
    kArray,
    kObject,
  };

  Type type{Type::kNull};
  bool boolean{false};
  double number{0.0};
  std::string string;
  std::vector<Json> array;
  std::vector<std::pair<std::string, Json>> object;
};

class JsonParser
{
public:
  explicit JsonParser(const char * text)
  : p_(text) {}

  Json parse()
  {
    skipWhitespace();
    Json value = parseValue();
    skipWhitespace();
    if (*p_ != '\0') {
      fail("trailing characters after JSON value");
    }
    return value;
  }

private:
  [[noreturn]] void fail(const char * message) const
  {
    throw std::runtime_error(std::string("json: ") + message);
  }

  void skipWhitespace()
  {
    while (*p_ == ' ' || *p_ == '\t' || *p_ == '\r' || *p_ == '\n') {
      ++p_;
    }
  }

  Json parseValue()
  {
    switch (*p_) {
      case '{': return parseObject();
      case '[': return parseArray();
      case '"': return parseString();
      case 't':
        expectLiteral("true");
        return Json{Json::Type::kBool, true, 0.0, "", {}, {}};
      case 'f':
        expectLiteral("false");
        return Json{Json::Type::kBool, false, 0.0, "", {}, {}};
      case 'n':
        expectLiteral("null");
        return Json{};
      default:
        if (*p_ == '-' || (*p_ >= '0' && *p_ <= '9')) {
          return parseNumber();
        }
        fail("unexpected character");
    }
    // Unreachable; keeps -Wreturn-type quiet.
    return Json{};
  }

  void expectLiteral(const char * literal)
  {
    const std::size_t n = std::strlen(literal);
    if (std::strncmp(p_, literal, n) != 0) {
      fail("bad literal");
    }
    p_ += n;
  }

  Json parseObject()
  {
    Json result{Json::Type::kObject, false, 0.0, "", {}, {}};
    ++p_;  // '{'
    skipWhitespace();
    if (*p_ == '}') {
      ++p_;
      return result;
    }
    for (;; ) {
      skipWhitespace();
      if (*p_ != '"') {
        fail("expected object key");
      }
      std::string key = parseString().string;
      skipWhitespace();
      if (*p_ != ':') {
        fail("expected ':' after object key");
      }
      ++p_;
      skipWhitespace();
      result.object.emplace_back(std::move(key), parseValue());
      skipWhitespace();
      if (*p_ == ',') {
        ++p_;
        continue;
      }
      if (*p_ == '}') {
        ++p_;
        return result;
      }
      fail("expected ',' or '}' in object");
    }
  }

  Json parseArray()
  {
    Json result{Json::Type::kArray, false, 0.0, "", {}, {}};
    ++p_;  // '['
    skipWhitespace();
    if (*p_ == ']') {
      ++p_;
      return result;
    }
    for (;; ) {
      skipWhitespace();
      result.array.push_back(parseValue());
      skipWhitespace();
      if (*p_ == ',') {
        ++p_;
        continue;
      }
      if (*p_ == ']') {
        ++p_;
        return result;
      }
      fail("expected ',' or ']' in array");
    }
  }

  Json parseString()
  {
    Json result{Json::Type::kString, false, 0.0, "", {}, {}};
    ++p_;  // '"'
    std::string & out = result.string;
    while (*p_ != '\0' && *p_ != '"') {
      const char c = *p_;
      if (c == '\\') {
        ++p_;
        switch (*p_) {
          case '"': out.push_back('"'); ++p_; break;
          case '\\': out.push_back('\\'); ++p_; break;
          case '/': out.push_back('/'); ++p_; break;
          case 'b': out.push_back('\b'); ++p_; break;
          case 'f': out.push_back('\f'); ++p_; break;
          case 'n': out.push_back('\n'); ++p_; break;
          case 'r': out.push_back('\r'); ++p_; break;
          case 't': out.push_back('\t'); ++p_; break;
          case 'u': {
              ++p_;
              const unsigned int cp = parseHex4();
            // Only BMP code points; sufficient for keys in this schema.
              out.push_back(static_cast<char>(0xC0u | ((cp >> 6u) & 0x1Fu)));
              out.push_back(static_cast<char>(0x80u | (cp & 0x3Fu)));
              break;
            }
          default: fail("bad escape");
        }
      } else {
        out.push_back(c);
        ++p_;
      }
    }
    if (*p_ != '"') {
      fail("unterminated string");
    }
    ++p_;
    return result;
  }

  unsigned int parseHex4()
  {
    unsigned int value = 0u;
    for (int i = 0; i < 4; ++i) {
      const char c = *p_;
      unsigned int digit = 0u;
      if (c >= '0' && c <= '9') {
        digit = static_cast<unsigned int>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        digit = static_cast<unsigned int>(c - 'a') + 10u;
      } else if (c >= 'A' && c <= 'F') {
        digit = static_cast<unsigned int>(c - 'A') + 10u;
      } else {
        fail("bad \\u escape");
      }
      value = (value << 4u) | digit;
      ++p_;
    }
    return value;
  }

  Json parseNumber()
  {
    // strtod consumes the longest valid prefix; the span check below rejects
    // garbage like "1.2.3".
    char * end = nullptr;
    const double value = std::strtod(p_, &end);
    if (end == p_) {
      fail("bad number");
    }
    p_ = end;
    return Json{Json::Type::kNumber, false, value, "", {}, {}};
  }

  const char * p_;
};

// -- accessors --------------------------------------------------------------

const Json & objectField(const Json & value, const char * key)
{
  if (value.type != Json::Type::kObject) {
    throw std::runtime_error(std::string("expected object, got non-object"));
  }
  for (const auto & field : value.object) {
    if (field.first == key) {
      return field.second;
    }
  }
  throw std::runtime_error(std::string("missing field '") + key + "'");
}

bool hasField(const Json & value, const char * key)
{
  if (value.type != Json::Type::kObject) {
    return false;
  }
  for (const auto & field : value.object) {
    if (field.first == key) {
      return true;
    }
  }
  return false;
}

std::vector<double> numberArray(const Json & value)
{
  if (value.type != Json::Type::kArray) {
    throw std::runtime_error("expected numeric array");
  }
  std::vector<double> out;
  out.reserve(value.array.size());
  for (const Json & item : value.array) {
    if (item.type != Json::Type::kNumber) {
      throw std::runtime_error("expected number in array");
    }
    out.push_back(item.number);
  }
  return out;
}

// -- output -----------------------------------------------------------------

void appendEscaped(std::string & out, const std::string & text)
{
  out.push_back('"');
  for (const char c : text) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out.push_back(c); break;
    }
  }
  out.push_back('"');
}

void appendNumber(std::string & out, double value)
{
  char buffer[32];
  // %.17g round-trips a double exactly; the parity assertion is 1e-6.
  std::snprintf(buffer, sizeof(buffer), "%.17g", value);
  out += buffer;
}

// -- scenario (fixed, mirrors the gtest fixture) ------------------------------

MpcConfig scenarioConfig()
{
  MpcConfig mpc;
  mpc.model = ModelType::kDoubleIntegrator2D;
  mpc.horizon = 8;
  mpc.dt = 0.1;
  mpc.Q = (Eigen::VectorXd(4) << 10.0, 10.0, 1.0, 1.0).finished();
  mpc.R = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
  mpc.Qf = (Eigen::VectorXd(4) << 100.0, 100.0, 10.0, 10.0).finished();
  mpc.x_min = (Eigen::VectorXd(4) << -1.0e9, -1.0e9, -2.0, -2.0).finished();
  mpc.x_max = (Eigen::VectorXd(4) << 1.0e9, 1.0e9, 2.0, 2.0).finished();
  mpc.u_min = (Eigen::VectorXd(2) << -1.0, -1.0).finished();
  mpc.u_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
  return mpc;
}

CbfConfig scenarioCbf()
{
  CbfConfig cbf;
  cbf.variant = CbfVariant::kFixedDecay;
  cbf.gamma = 0.3;
  cbf.cbf_horizon = 0;  // full horizon
  cbf.ego_radius = 0.15;
  cbf.safety_margin = 0.05;
  return cbf;
}

// acados' C code writes solver diagnostics ("QP solver returned error status
// ... ACADOS_MINSTEP") to stdout. This CLI's contract is one JSON object per
// line on stdout, so that chatter would corrupt the stream the parity test
// parses — and it only appears on the hard states, which is exactly where the
// comparison matters. Redirect fd 1 to /dev/null for the duration of the solve
// and restore it before writing the response. stderr is left alone so real
// errors stay visible.
class StdoutSilencer
{
public:
  StdoutSilencer()
  {
    std::fflush(stdout);
    saved_fd_ = ::dup(STDOUT_FILENO);
    const int devnull = ::open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      ::dup2(devnull, STDOUT_FILENO);
      ::close(devnull);
    }
  }

  ~StdoutSilencer()
  {
    std::fflush(stdout);
    if (saved_fd_ >= 0) {
      ::dup2(saved_fd_, STDOUT_FILENO);
      ::close(saved_fd_);
    }
  }

  StdoutSilencer(const StdoutSilencer &) = delete;
  StdoutSilencer & operator=(const StdoutSilencer &) = delete;

private:
  int saved_fd_{-1};
};

const char * statusName(SolverStatus status)
{
  switch (status) {
    case SolverStatus::kSuccess: return "SUCCESS";
    case SolverStatus::kMaxIterations: return "MAX_ITERATIONS";
    case SolverStatus::kQpFailure: return "QP_FAILURE";
    case SolverStatus::kInfeasible: return "INFEASIBLE";
    case SolverStatus::kNanDetected: return "NAN_DETECTED";
    case SolverStatus::kNotInitialized: return "NOT_INITIALIZED";
  }
  return "UNKNOWN";
}

Eigen::VectorXd toVectorXd(const std::vector<double> & values)
{
  Eigen::VectorXd out(static_cast<Eigen::Index>(values.size()));
  for (std::size_t i = 0; i < values.size(); ++i) {
    out[static_cast<Eigen::Index>(i)] = values[i];
  }
  return out;
}

}  // namespace

int main()
{
  // One solver for every line: the process is meant to run many solves.
  MpcCbfSolver solver(scenarioConfig(), scenarioCbf());
  const bool initialized = solver.initialize();

  std::string line;
  while (std::getline(std::cin, line)) {
    // Trim trailing '\r' (Windows line endings in some editors).
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }

    std::string response;
    try {
      JsonParser parser(line.c_str());
      const Json input = parser.parse();

      const std::vector<double> x0v = numberArray(objectField(input, "x0"));
      if (x0v.size() != 4u) {
        throw std::runtime_error("x0 must have 4 entries (double integrator)");
      }
      Eigen::VectorXd x0 = toVectorXd(x0v);

      Eigen::VectorXd x_ref(4);
      x_ref << 1.0, 1.0, 0.0, 0.0;  // default goal, same as the fixture
      if (hasField(input, "x_ref")) {
        const std::vector<double> ref = numberArray(objectField(input, "x_ref"));
        if (ref.size() != 4u) {
          throw std::runtime_error("x_ref must have 4 entries");
        }
        x_ref = toVectorXd(ref);
      }
      solver.setReference(x_ref);

      std::vector<ObstacleState> obstacles;
      if (hasField(input, "obstacles")) {
        const Json & list = objectField(input, "obstacles");
        if (list.type != Json::Type::kArray) {
          throw std::runtime_error("obstacles must be an array");
        }
        obstacles.reserve(list.array.size());
        for (const Json & item : list.array) {
          ObstacleState obs;
          const std::vector<double> pos = numberArray(objectField(item, "position"));
          const std::vector<double> vel = numberArray(objectField(item, "velocity"));
          if (pos.size() != 3u || vel.size() != 3u) {
            throw std::runtime_error("position/velocity must have 3 entries");
          }
          obs.position = toVectorXd(pos);
          obs.velocity = toVectorXd(vel);
          obs.radius = objectField(item, "radius").number;
          obs.is_dynamic = objectField(item, "is_dynamic").boolean;
          obstacles.push_back(obs);
        }
      }

      if (!initialized) {
        std::printf("{\"status\": \"NOT_INITIALIZED\"}\n");
        continue;
      }

      // One process serves every request, so without this the solver would
      // carry the shifted warm start from the previous — unrelated — state.
      // The Python side solves each grid point as an independent problem from
      // the u = 0 coasting rollout; a stale warm start is the documented route
      // to ACADOS_MINSTEP at the first SQP linearization, so the two would not
      // be solving the same problem. Coast here too: constant-x0 on stages
      // 1..N, u = 0 everywhere.
      solver.reset();
      {
        const int horizon = solver.mpcConfig().horizon;
        Eigen::MatrixXd x_guess(solver.stateDim(), horizon + 1);
        x_guess.colwise() = x0;
        solver.warmStart(
          x_guess, Eigen::MatrixXd::Zero(solver.inputDim(), horizon));
      }

      MpcCbfSolution sol;
      {
        StdoutSilencer silence_acados;
        sol = solver.solve(x0, obstacles);
      }
      const SolverDiagnostics & diag = sol.diagnostics;

      response = "{\"status\": \"";
      response += statusName(sol.status);
      response += "\", \"u0\": [";
      for (int i = 0; i < sol.u0.size(); ++i) {
        if (i > 0) {
          response += ", ";
        }
        appendNumber(response, sol.u0[static_cast<Eigen::Index>(i)]);
      }
      response += "], \"solve_time_ms\": ";
      appendNumber(response, diag.solve_time_ms);
      response += ", \"cbf_values\": [";
      for (std::size_t i = 0; i < diag.cbf_values.size(); ++i) {
        if (i > 0) {
          response += ", ";
        }
        appendNumber(response, diag.cbf_values[i]);
      }
      response += "], \"first_active_cbf_step\": ";
      response += std::to_string(diag.first_active_cbf_step);
      response += ", \"first_active_obstacle\": ";
      response += std::to_string(diag.first_active_obstacle);
      response += ", \"infeasibility_reason\": ";
      appendEscaped(response, diag.infeasibility_reason);
      response += "}";
    } catch (const std::exception & e) {
      response = "{\"status\": \"ERROR\", \"error\": ";
      appendEscaped(response, e.what());
      response += "}";
    }

    std::printf("%s\n", response.c_str());
  }

  return EXIT_SUCCESS;
}
