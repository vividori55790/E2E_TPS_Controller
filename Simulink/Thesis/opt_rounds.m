function opt_rounds(r0, r1, py, fams)
% opt_rounds.m -- OPT follow-up rounds without a loop turn in between: for rnd = r0..r1, `surface_opt_check.py next`
% writes the points of the configurations whose measured powers do not bracket |P| yet (out/opt_points_<fam>_r<rnd>.csv),
% then opt_chain(rnd) runs them. Stops at the first round with nothing to run. Restart-safe like opt_chain.
% fams (optional, 10-30): cell array of families, e.g. {'ms','msk1'} for the OPT4 M-S verification rounds; default = the
% OPT1-3 families of `next` (mb, k1, grid).
%   python automation/job.py submit optR --interactive --after rob1 -- matlab -batch "cd('Simulink/Thesis'); opt_rounds(2, 4, '<python.exe>')"
%   python automation/job.py submit msR --interactive --after rob3 -- matlab -batch "cd('Simulink/Thesis'); opt_rounds(2, 4, '<python.exe>', {'ms','msk1'})"
    if nargin < 1 || isempty(r0), r0 = 2; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    if nargin < 4, fams = {}; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    arg = '';
    if ~isempty(fams), arg = [' --fams ', strjoin(fams, ',')]; end
    for rnd = r0:r1
        [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next%s', py, fullfile(here, 'surface_opt_check.py'), arg));
        fprintf('%s', msg);
        if st ~= 0, error('opt_rounds: surface_opt_check.py next failed (exit %d)', st); end
        pending = false;
        if isempty(fams)
            pending = ~isempty(dir(fullfile(out, sprintf('opt_points_*_r%d.csv', rnd))));
        else
            for i = 1:numel(fams)
                pending = pending || isfile(fullfile(out, sprintf('opt_points_%s_r%d.csv', fams{i}, rnd)));
            end
        end
        if ~pending
            fprintf('opt_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        if isempty(fams)
            opt_chain(rnd);
        else
            opt_chain(rnd, fams);
        end
    end
    fprintf('opt_rounds: stopped after round %d %s\n', r1, datestr(now));
end
