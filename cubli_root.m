function r = cubli_root()
%CUBLI_ROOT 项目根目录。所有会写生成文件的脚本都用它定位输出位置。
%
% 它替换掉了原先散在 140 个脚本里的
%     root = fileparts(mfilename('fullpath'));
% 那句在**扁平目录**下是对的——所有脚本都在同一个文件夹里，所以脚本自己所在的
% 文件夹就是项目根。脚本一旦搬进 src/<角色>/，那句话就变成了**子文件夹**，
% 于是生成的 .slx 会散进 src/core、src/diagnostics……而不是项目根。
%
% 本文件就放在项目根，所以在这里 `fileparts(mfilename('fullpath'))` **就是**
% 项目根——这是全项目唯一一处仍旧该用那个写法的地方。
%
% 要改项目布局，只改这一个文件。

r = fileparts(mfilename('fullpath'));
end
