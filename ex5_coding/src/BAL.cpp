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
  string input_file, out_b4_file, out_af_file;
  g2o::CommandArgs arg;
  arg.param("i", input_file, "", "input text file from BAL dataset");
  arg.param("b", out_b4_file, "out_before.ply", "points before BA in ply format");
  arg.param("o", out_af_file, "out_after.ply", "points after BA in ply format");
  arg.param("n", max_iter, 40, "perform n iterations");
  arg.parseArgs(argc, argv);
  if(input_file.empty()){
    arg.printHelp(cout);
    return 1;
  }
  
  // init BAL dataset
  bal_dataset bal_ds;
  // parse BAL dataset
  bal_ds.read_from_text(input_file);
  bal_ds.write_to_ply(out_b4_file);
  // setup optimizer
  bal_ds.init_opt();
  // run optimization
  bal_ds.run_opt(max_iter);
  // dump the points
  bal_ds.write_to_ply(out_af_file);

  return 0;
}
