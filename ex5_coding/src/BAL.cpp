// bundle adjustment in large (BAL)
// 29.05.2018 by Min-An Chao
// adjusted from g2o github by R. Kümmerle
// and from slambook github by Xiang Gao

#include <iostream>
using namespace std;

// g2o cammand parsing
#include "g2o/stuff/command_args.h"

// class definition
#include "def_bal.hpp"

int main(int argc, char** argv)
{
  int max_iter;
  string input_file, snapshot_file, output_file;
  g2o::CommandArgs arg;
  arg.param("i", input_file, "", "input text file from BAL dataset");
  arg.param("o", output_file, "out.wrl", "write final points into a vrml file");
  arg.param("s", snapshot_file, "snap.wrl", "write points before optimization into a vrml file");
  arg.param("n", max_iter, 30, "perform n iterations");
  arg.parseArgs(argc, argv);
  
  // init BAL dataset
  bal_dataset bal_ds;
  // parse BAL dataset
  bal_ds.read_from_text(input_file);
  bal_ds.write_to_vrml(snapshot_file);
  // setup optimizer
  bal_ds.init_opt();
  // run optimization
  bal_ds.run_opt(max_iter);
  // dump the points
  bal_ds.write_to_vrml(output_file);

  return 0;
}
