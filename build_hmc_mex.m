function build_hmc_mex
%BUILD_HMC_MEX Compile the optional native independent-vehicle accelerator.
% A supported C++ compiler must already be selected by mex -setup C++.
here=fileparts(mfilename('fullpath'));
source=fullfile(here,'hmc_mex.cpp');
compiler=mex.getCompilerConfigurations('C++','Selected');
assert(~isempty(compiler),'HMC:CompilerMissing', ...
    'Select a supported C++ compiler first by running: mex -setup C++');
if ispc && contains(lower(compiler.Manufacturer),'microsoft')
    mex('-R2018a','COMPFLAGS=$COMPFLAGS /std:c++17', ...
        '-outdir',here,'-output','hmc_mex',source);
else
    mex('-R2018a','CXXFLAGS=$CXXFLAGS -std=c++17', ...
        '-outdir',here,'-output','hmc_mex',source);
end
fprintf('Built hmc_mex in %s\n',here);
end
