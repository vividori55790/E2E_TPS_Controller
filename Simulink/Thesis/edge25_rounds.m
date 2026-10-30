function edge25_rounds(r0, r1, py)
% edge25_rounds.m -- the ends of the specified power range on the 25 ns grid (edge25.py) in rounds without a loop turn in
% between, as rev25_rounds.m does for LV -> HV. Round 1 runs out/edge25_points_r1.csv (from `edge25.py make`; an error if it is
% missing, which is how a queued job is stopped); every later round first runs `edge25.py next`, which writes
% out/edge25_points_r<rnd>.csv with more D0 levels for the configurations whose powers do not bracket |P| yet, and stops at the
% first round with nothing to run. verify_steady_fine_batch: 12 runs per chunk, 20 ms; restart-safe.
%   python automation/job.py submit edge25 --interactive --after flip25 -- matlab -batch "cd('Simulink/Thesis'); edge25_rounds(1, 4, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 1; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    for rnd = r0:r1
        pts = fullfile(out, sprintf('edge25_points_r%d.csv', rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'edge25.py')));
            fprintf('%s', msg);
            if st ~= 0, error('edge25_rounds: edge25.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            error('edge25_rounds: %s not found (run `edge25.py make` first)', pts);
        end
        if ~isfile(pts)
            fprintf('edge25_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        verify_steady_fine_batch(pts, fullfile(out, sprintf('edge25_sweep_r%d.csv', rnd)), 12, 0.020);
    end
    fprintf('edge25_rounds: stopped after round %d %s\n', r1, datestr(now));
end
