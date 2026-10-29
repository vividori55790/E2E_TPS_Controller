function opt_rounds(r0, r1, py)
% opt_rounds.m -- OPT follow-up rounds without a loop turn in between: for rnd = r0..r1, `surface_opt_check.py next`
% writes the points of the configurations whose measured powers do not bracket |P| yet (out/opt_points_<fam>_r<rnd>.csv),
% then opt_chain(rnd) runs them. Stops at the first round with nothing to run. Restart-safe like opt_chain.
%   python automation/job.py submit optR --interactive --after rob1 -- matlab -batch "cd('Simulink/Thesis'); opt_rounds(2, 4, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 2; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    for rnd = r0:r1
        [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'surface_opt_check.py')));
        fprintf('%s', msg);
        if st ~= 0, error('opt_rounds: surface_opt_check.py next failed (exit %d)', st); end
        if isempty(dir(fullfile(out, sprintf('opt_points_*_r%d.csv', rnd))))
            fprintf('opt_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        opt_chain(rnd);
    end
    fprintf('opt_rounds: stopped after round %d %s\n', r1, datestr(now));
end
