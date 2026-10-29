function opt_chain(rnd)
% opt_chain.m -- OPT1-OPT3 Simulink runs (surface_opt_check.py) in ONE MATLAB session, one family after another:
%   mb   (OPT3, M-B local cross + SPS + BT)   out/opt_points_mb_r<rnd>.csv   -> out/opt_sweep_mb_r<rnd>_partNNN.csv   30 ms
%   k1   (OPT2, k = 1 valley test, 95 V)      out/opt_points_k1_r<rnd>.csv   -> out/opt_sweep_k1_r<rnd>_partNNN.csv   40 ms
%   grid (OPT1, R6 best combos in steady st.) out/opt_points_grid_r<rnd>.csv -> out/opt_sweep_grid_r<rnd>_partNNN.csv 30 ms
% A family whose points file does not exist is skipped. verify_steady_batch skips chunks already on disk, so the job can
% be resubmitted after a reboot. An error in one family does not stop the others; the job ends with an error if any failed.
%   python automation/job.py submit opt1 --interactive -- matlab -batch "cd('Simulink/Thesis'); opt_chain(1)"
    if nargin < 1, rnd = 1; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    fprintf('opt_chain: round %d start %s\n', rnd, datestr(now));
    fams = {'mb', 0.030; 'k1', 0.040; 'grid', 0.030};
    failed = {};
    for i = 1:size(fams, 1)
        fam = fams{i, 1};
        fp = fullfile(out, sprintf('opt_points_%s_r%d.csv', fam, rnd));
        if ~isfile(fp)
            fprintf('--- %s: %s not found, skipped\n', fam, fp);
            continue;
        end
        fprintf('--- %s (%g ms) %s\n', fam, fams{i, 2} * 1e3, datestr(now));
        fo = fullfile(out, sprintf('opt_sweep_%s_r%d.csv', fam, rnd));
        try
            verify_steady_batch(fp, fo, 24, fams{i, 2});
        catch ME
            fprintf('%s FAILED: %s\n', fam, getReport(ME, 'extended', 'hyperlinks', 'off'));
            failed{end + 1} = fam; %#ok<AGROW>
        end
    end
    fprintf('opt_chain: done %s\n', datestr(now));
    if ~isempty(failed), error('opt_chain: %s failed (reports above)', strjoin(failed, ', ')); end
end
