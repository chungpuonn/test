" Basic Setup
""""""""""""""""""
filetype on 			"detect file types
filetype plugin indent on 	"user different indent for different file type
filetype plugin on 		"use plugins for file
filetype indent on
set nocompatible 		"disable vi compatibility
" set clipoard=unnamed "Yanks go on clipboard instead
set novisualbell "no blinking
set autoread			"autoload file after changed
set title 			"change the title of the terminal
set showmatch			"show matched brackets
set magic
set mouse-=a "disable Mouse
set mousehide "hide mouse after chars typed
set splitbelow			"split current window and start new one below
set splitright			"split current window and start a new on on the right

" Status
"""""""""""""""""""
setl number			"show number
set nu
set ruler 			"show ruler
set relativenumber		"show relative line number
set showcmd			"display an incomplete command in the lower right corner of the 				window
set laststatus=2		"always show command
set statusline+=%F		"show full path of current file
au InsertEnter * hi Normal ctermbg=234 guibg=#000000
au InsertLeave * hi Normal ctermbg=232 guibg=#1b1d1e
" Search
"""""""""""""""""""""
set hlsearch 			"heighlight serch result
set incsearch			"search with input

" Fold
""""""""""""""""""""
set foldenable			"enable folding
set foldmethod=syntax		"fold 
set foldlevel=99

" Encoding 
"""""""""""""""""""
set encoding=utf-8

" Syntax
""""""""""""""""""
syntax enable			"highlight based on syntax
syntax on

" Visual 
""""""""""""""""""""""""""""""
set cursorline "highlight current 

" Plugins
set rtp+=~/.vim/bundle/Vundle.vim
call vundle#begin()

Plugin 'VundleVim/Vundle.vim'
Plugin 'scrooloose/nerdtree'
"Plugin 'Scrooloose/syntastic'
Plugin 'vim-airline/vim-airline'
Plugin 'vim-airline/vim-airline-themes'
Plugin 'ervandew/Supertab'
Plugin 'tpope/vim-fugitive'
Plugin 'tpope/vim-eunuch.git'
Plugin 'plasticboy/vim-markdown'
"Plugin 'justmao945/vim-clang'
"Plugin 'benmills/vimux'
"Plugin 'lervag/vimtex'  "vim support for tex
Plugin 'scrooloose/nerdcommenter'
Plugin 'crusoexia/vim-monokai'
Plugin 'octol/vim-cpp-enhanced-highlight'
Bundle 'DoxygenToolkit.vim'
Bundle 'altercation/vim-colors-solarized'
Plugin 'ycm-core/YouCompleteMe'
call vundle#end()


" Plugin Settings
"""""""""""""""""""""""""
" Airline
let g:Powerline_symbols = 'fancy'
let g:airline_detect_whitespace=1
let g:airline_section_z = '%{strftime("%c")}'
let g:airline_section_y = 'BN: %{bufnr("%")}'
"let g:airline_theme='monokai'
let g:airline#extensions#branch#enabled = 1
let g:airline#extensions#tabline#enabled = 1
let g:airline#extensions#tabline#fnamemod = ':.'
let g:airline#extensions#tabline#fnamecollapse = 0
let g:airline#extensions#whitespace#enabled = 0
let g:Powerline_symbols = 'fancy'
let g:airline_powerline_fonts=1
if !exists('g:airline_symbols')
		let g:airline_symbols = {}
endif
let g:airline#extensions#tabline#buffer_nr_show = 1
    " unicode symbols
"let g:airline_left_sep = '»'
"let g:airline_left_sep = '▶'
"let g:airline_right_sep = '«'
"let g:airline_right_sep = '◀'
"let g:airline_symbols.linenr = '␊'
"let g:airline_symbols.linenr = '␤'
"let g:airline_symbols.linenr = '¶'
"let g:airline_symbols.branch = '⎇'
"let g:airline_symbols.paste = 'ρ'
"let g:airline_symbols.paste = 'Þ'
"let g:airline_symbols.paste = '∥'
"let g:airline_symbols.whitespace = ''
    
    "" " airline symbols
"let g:airline_left_sep = ''
"let g:airline_left_alt_sep = ''
"let g:airline_right_sep=''
"let g:airline_right_alt_sep = ''
"let g:airline_symbols.branch = ''
"let g:airline_symbols.readonly = ''
"let g:airline_symbols.linenr = ''
"
""""""""""""""""""""""""
" Nerd Tree
""""""""""""""""""""""""
let NERDTreeHighlightCursorline = 1
let NERDTreeIgnore=['\.pyc$','\.pyo$']
let g:netrw_homw = '~/'

map <C-n> :NERDTreeToggle<CR>        " use ctrl+n to toggle nerdtree
map <C-o> :NERDTree %<CR>            " use ctrl+o to enter current file directory for nerdtree
let NERDTreeIgnore=[ '\.pyc$', '\.pyo$', '\.obj$', '\.o$', '\.so$', '\.egg$','^\.git$', '^\.svn$', '^\.hg$' ]
let g:netrw_home='~/'

" close vim if the only window left open is a NERDTree
autocmd bufenter * if (winnr("$") == 1 && exists("b:NERDTreeType") && b:NERDTreeType == "primary") | q | end

"color definnation for nerdtree 
function! NERDTreeHighlightFile(extension, fg, bg, guifg, guibg)
	exec 'autocmd filetype nerdtree  syn match ' . a:extension .' #^\s\+.*'. a:extension .'$#'
	exec 'autocmd filetype nerdtree highligh ' . a:extension .' ctermbg='. a:bg .' ctermfg='. a:fg .' guibg='. a:guibg .' guifg='. a:guifg

endfunction

call NERDTreeHighlightFile('m','Red','none','red','#151515')
call NERDTreeHighlightFile('jade', 'green', 'none', 'green', '#151515')
call NERDTreeHighlightFile('ini', 'yellow', 'none', 'yellow', '#151515')
call NERDTreeHighlightFile('md', 'blue', 'none', '#3366FF', '#151515')
call NERDTreeHighlightFile('yml', 'yellow', 'none', 'yellow', '#151515')
call NERDTreeHighlightFile('config', 'yellow', 'none', 'yellow', '#151515')
call NERDTreeHighlightFile('conf', 'yellow', 'none', 'yellow', '#151515')
call NERDTreeHighlightFile('json', 'yellow', 'none', 'yellow', '#151515')
call NERDTreeHighlightFile('html', 'yellow', 'none', 'yellow', '#151515')
call NERDTreeHighlightFile('styl', 'cyan', 'none', 'cyan', '#151515')
call NERDTreeHighlightFile('css', 'cyan', 'none', 'cyan', '#151515')
call NERDTreeHighlightFile('coffee', 'Red', 'none', 'red', '#151515')
call NERDTreeHighlightFile('js', 'Red', 'none', '#ffa500', '#151515')
call NERDTreeHighlightFile('php', 'Magenta', 'none', '#ff00ff', '#151515')

" Doxygen 
""""""""""""""""""""""""
nnoremap <leader>dd :Dox<cr>
nnoremap <leader>da :DoxAuthor<cr>
nnoremap <leader>db :DoxBlock<cr>
nnoremap <leader>dl :DoxLic<cr>
let g:DoxygenToolkit_authorName="Bingbing Li, bingbing.li@ntu.edu.sg"
let g:DoxygenToolkit_briefTag_funcName="yes"
let g:doxygen_enhanced_color=1
let g:load_doxygen_syntax=1

"YCM settings 
""""""""""""""""""""""""""""""""""""""""
let g:ycm_show_diagnostics_ui=1
let g:ycm_complete_in_comments=1   "show completeion even in comments
let g:ycm_error_symbol='e>'  "change error indicator to e>
let g:ycm_warning_symbol='w>'  "change warrning indicator to w>
let g:ycm_semantic_triggers = {
\   'roslaunch' : ['="', '$(', '/'],
\   'rosmsg,rossrv,rosaction' : ['re!^', '/'],
\}
"let g:ycm_global_ycm_extra_conf='~/.ycm_extra_config.py'
let g:ycm_echo_current_diagnostic=1

" Auto commands
"""""""""""""""""""""""""""""""""""
au BufRead,BufNewFile {*.md,*.mkd,*.markdown}                         set ft=markdown
au BufRead,BufNewFile {COMMIT_EDITMSG}				      set ft=gitcommit
" The following line enables the nerdtree plugin for VIm
"autocmd vimenter * NERDTree
autocmd! bufwritepost .vimrc source % "auto load vimrc after change it

" Shortcurts 
""""""""""""""""""""""""""""""""""""
map <leader>q :wq!<cr>	
" use \q to save and quit
map <F7> :tabp<cr> 
" use F7 to swap to previsous tab page
map <F8> :tabn<cr>
" use F8 to swap to next tab page 
nnoremap <leader>[ :bprevious<cr>
" use \[ to move to previsous buffer
nnoremap <leader>] :bnext<cr>
" use \] to move to next buffer
inoremap <expr><CR> pumvisible() ? "\<C-y>" : "\<CR>" 
" use Enter to select current suggest 
"shortcut for YCM
noremap <leader>yf :YcmCompleter FixIt<cr>
noremap <leader>yd :YcmDiags<cr>
" use \yf and \yd to fix the error using YCM