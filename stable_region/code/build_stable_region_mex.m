function binary=build_stable_region_mex
%BUILD_STABLE_REGION_MEX Build the supplied region observer, not new physics.
% GNU Octave/Linux: requires mkoctfile and a GCC C++17/OpenMP toolchain.
% MATLAB/Linux: select a release-supported GCC with mex -setup C++.
% MATLAB/Windows: select a release-supported Microsoft C++ compiler.
% MATLAB build branches are provided but were not executed/validated here.
% macOS/Clang OpenMP requires separate toolchain configuration and is rejected.
% The supplied source includes root hmc_mex.cpp/hmc_native.cpp unchanged.
% Uses -O2/C++17/OpenMP; no fast-math flags or numerical model changes.
codeDir=fileparts(mfilename('fullpath'));
root=fileparts(fileparts(codeDir));
source=fullfile(codeDir,'hmc_region_mex.cpp');
assert(isfile(source),'HMC:RegionSourceMissing','Missing supplied source: %s',source);
assert(isfile(fullfile(root,'hmc_mex.cpp'))&&isfile(fullfile(root,'hmc_native.cpp'))&& ...
    isfile(fullfile(root,'tire_fields.inc')),'HMC:RegionIncludeMissing', ...
    'The original HMC C++ sources must remain in the repository root.');
assert(~mislocked('hmc_region_mex'),'HMC:RegionModuleInUse', ...
    'Close active region model handles before rebuilding the module.');
clear hmc_region_mex;
binary=fullfile(codeDir,['hmc_region_mex.' mexext]);
include=['-I' root];
if exist('OCTAVE_VERSION','builtin')~=0
    assert(isunix&&~ismac,'HMC:RegionBuildPlatform', ...
        'This Octave build path targets Linux with GCC/OpenMP.');
    oldCXX=getenv('CXXFLAGS');oldLD=getenv('LDFLAGS');
    restore=onCleanup(@()restore_flags(oldCXX,oldLD)); %#ok<NASGU>
    [cxx,status]=mkoctfile('-p','CXXFLAGS');
    assert(status==0,'HMC:RegionCompilerQuery','Cannot query mkoctfile CXXFLAGS.');
    [ld,status]=mkoctfile('-p','LDFLAGS');
    assert(status==0,'HMC:RegionCompilerQuery','Cannot query mkoctfile LDFLAGS.');
    setenv('CXXFLAGS',[cxx ' -std=c++17 -O2 -fopenmp']);
    setenv('LDFLAGS',[ld ' -fopenmp']);
    [output,status]=mkoctfile('--mex',include,source,'-o',binary);
    if ~isempty(output),fprintf('%s\n',output);end
    assert(status==0,'HMC:RegionBuildFailed','mkoctfile failed; see compiler output.');
else
    assert(~ismac,'HMC:RegionBuildPlatform', ...
        'macOS OpenMP needs an explicitly configured compiler and libomp.');
    compiler=mex.getCompilerConfigurations('C++','Selected');
    assert(~isempty(compiler),'HMC:RegionCompilerMissing','Run mex -setup C++ first.');
    if ispc&&contains(lower(compiler.Manufacturer),'microsoft')
        mex('-R2018a','COMPFLAGS=$COMPFLAGS /std:c++17 /O2 /openmp', ...
            include,'-outdir',codeDir,'-output','hmc_region_mex',source);
    elseif isunix&&contains(lower(compiler.Manufacturer),'gnu')
        mex('-R2018a','CXXFLAGS=$CXXFLAGS -std=c++17 -O2 -fopenmp', ...
            'LDFLAGS=$LDFLAGS -fopenmp',include, ...
            '-outdir',codeDir,'-output','hmc_region_mex',source);
    else
        error('HMC:RegionCompilerUnsupported', ...
            'Use a release-supported Linux GCC or Windows Microsoft C++ compiler.');
    end
end
assert(isfile(binary),'HMC:RegionBuildOutput','The compiler did not create %s.',binary);
fprintf('Built %s\nSHA-256: %s\n',binary,stable_region_sha256(binary));
end

function restore_flags(cxx,ld)
setenv('CXXFLAGS',cxx);setenv('LDFLAGS',ld);
end
