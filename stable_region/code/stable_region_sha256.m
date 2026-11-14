function out=stable_region_sha256(path)
%STABLE_REGION_SHA256 SHA-256 of exact file bytes in MATLAB or GNU Octave.
fid=fopen(path,'rb');
assert(fid>=0,'HMC:RegionChecksum','Cannot open %s for checksum.',path);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
bytes=fread(fid,Inf,'*uint8');
if exist('OCTAVE_VERSION','builtin')~=0
    out=hash('sha256',char(bytes(:).'));
else
    md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
    md.update(typecast(bytes(:),'int8'));raw=md.digest();
    out=lower(reshape(dec2hex(mod(double(raw),256),2).',1,[]));
end
end
