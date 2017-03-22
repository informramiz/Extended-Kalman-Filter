#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void CheckArguments(int argc, char* argv[]);
void ReadMeasurements(string in_file_name, vector<MeasurementPackage> &measurement_pack_list,
                       vector<GroundTruthPackage>& gt_pack_list);
void CheckFiles(const string& in_name, const string& out_name);
void TestEkf(const string& in_file_name, const string& out_file_name);
void RunEkf(const vector<MeasurementPackage>& measurement_pack_list,
            const vector<GroundTruthPackage>& gt_pack_list,
            const string& out_file_name);

int main(int argc, char* argv[]) {
  //check validity of arguments
  CheckArguments(argc, argv);

  string in_file_name = argv[1];
  string out_file_name = argv[2];
  //validate files existence
  CheckFiles(in_file_name, out_file_name);

  TestEkf(in_file_name, out_file_name);

  return 0;
}

void TestEkf(const string& in_file_name, const string& out_file_name) {
  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;
  ReadMeasurements(in_file_name, measurement_pack_list, gt_pack_list);

  RunEkf(measurement_pack_list, gt_pack_list, out_file_name);
}

void RunEkf(const vector<MeasurementPackage>& measurement_pack_list,
            const vector<GroundTruthPackage>& gt_pack_list,
            const string& out_file_name) {

  ofstream out_file(out_file_name, ofstream::out);
  if (!out_file.is_open()) {
    std::cerr << "Error opening file: " << out_file_name << std::endl;
    exit(EXIT_FAILURE);
  }

  // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    VectorXd x = fusionEKF.GetEstimations();
    out_file << x(0) << "\t";
    out_file << x(1) << "\t";
    out_file << x(2) << "\t";
    out_file << x(3) << "\t";

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // output the estimation
      out_file << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file << ro * cos(phi) << "\t"; // p1_meas
      out_file << ro * sin(phi) << "\t"; // ps_meas
    }

    // output the ground truth packages
    out_file << gt_pack_list[k].gt_values_(0) << "\t";
    out_file << gt_pack_list[k].gt_values_(1) << "\t";
    out_file << gt_pack_list[k].gt_values_(2) << "\t";
    out_file << gt_pack_list[k].gt_values_(3) << "\n";

    estimations.push_back(fusionEKF.GetEstimations());
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

  if (out_file.is_open()) {
    out_file.close();
  }
}

void ReadMeasurements(string in_file_name, vector<MeasurementPackage> &measurement_pack_list,
                       vector<GroundTruthPackage>& gt_pack_list) {
  ifstream in_file(in_file_name.c_str(), ifstream::in);

  if (!in_file.is_open()) {
    std::cerr << "Can not open file: " << in_file << std::endl;
    exit(EXIT_FAILURE);
  }

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  string line;
  while (getline(in_file, line) && !in_file.eof()) {

    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x, y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT
      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }

  if (in_file.is_open()) {
    in_file.close();
  }
}

vector<MeasurementPackage> readMeasurements() {

  vector<MeasurementPackage> measurement_pack_list;

  // hardcoded input file with laser and radar measurements
  string in_file_name = "data/obj_pose-laser-radar-synthetic-input.txt";
  ifstream inFile(in_file_name.c_str(), std::ifstream::in);

  if(!inFile.is_open()) {
    std::cout << "Can not open input file: " << in_file_name << std::endl;
    return measurement_pack_list;
  }

  // set i to get only first 3 measurments
  int i = 0;
  string line;
  while(getline(inFile, line) && (i <= 5)) {
    MeasurementPackage measurement_pack;
    string sensor_type;
    long timestamp;

    istringstream iss(line); //reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) { //laser measurement
      //read measurements
      measurement_pack.sensor_type_ = MeasurementPackage::LASER;
      measurement_pack.raw_measurements_ = Eigen::VectorXd(2);

      float x;
      float y;

      iss >> x;
      iss >> y;
      iss >> timestamp;

      measurement_pack.raw_measurements_ << x, y;
      measurement_pack.timestamp_ = timestamp;

      measurement_pack_list.push_back(measurement_pack);
    } else if(sensor_type.compare("R") == 0) { //Radar measurement
      //read measurements
      measurement_pack.sensor_type_ = MeasurementPackage::RADAR;
      measurement_pack.raw_measurements_ = Eigen::VectorXd(3);

      float p;
      float phi;
      float p_dot;

      iss >> p;
      iss >> phi;
      iss >> p_dot;
      iss >> timestamp;

      measurement_pack.raw_measurements_ << p, phi, p_dot;
      measurement_pack.timestamp_ = timestamp;

      measurement_pack_list.push_back(measurement_pack);
      //          continue;
    }

    i++;
  }

  if(inFile.is_open()) {
    inFile.close();
  }

  return measurement_pack_list;
}

void CheckArguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void CheckFiles(const string& in_name, const string& out_name) {
  ifstream in_file(in_name.c_str(), ifstream::in);

  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }
  in_file.close();

  ofstream out_file(out_name.c_str(), ofstream::out);
  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
  out_file.close();
}
