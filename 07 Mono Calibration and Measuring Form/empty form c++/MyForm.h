#pragma once

#include <atlstr.h>
#include <Windows.h> 
#include <Eigen/Dense>
#include <cstdio>
#include <math.h>

using namespace Eigen;
using namespace System::IO;

LPCTSTR input, output;
int iWidth, iHeight;
long iSize;
BYTE* buffer;

int mouse_x, mouse_y;

int calib_x, calib_y;
int calib_count = 0;

float* world;
float* image;
float* projection = new float[12];

int p1x, p1y, p2x, p2y, p3x, p3y;
float w1x, w1y, w2x, w2y, w3x, w3y;
double distance;
double area;

BYTE* LoadBMP(int% width, int% height, long% size, LPCTSTR bmpfile)
{
	// declare bitmap structures
	BITMAPFILEHEADER bmpheader;
	BITMAPINFOHEADER bmpinfo;
	// value to be used in ReadFile funcs
	DWORD bytesread;
	// open file to read from
	HANDLE file = CreateFile(bmpfile, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_FLAG_SEQUENTIAL_SCAN, NULL);
	if (NULL == file)
		return NULL; // coudn't open file

					 // read file header
	if (ReadFile(file, &bmpheader, sizeof(BITMAPFILEHEADER), &bytesread, NULL) == false) {
		CloseHandle(file);
		return NULL;
	}
	//read bitmap info
	if (ReadFile(file, &bmpinfo, sizeof(BITMAPINFOHEADER), &bytesread, NULL) == false) {
		CloseHandle(file);
		return NULL;
	}
	// check if file is actually a bmp
	if (bmpheader.bfType != 'MB') {
		CloseHandle(file);
		return NULL;
	}
	// get image measurements
	width = bmpinfo.biWidth;
	height = abs(bmpinfo.biHeight);

	// check if bmp is uncompressed
	if (bmpinfo.biCompression != BI_RGB) {
		CloseHandle(file);
		return NULL;
	}
	// check if we have 24 bit bmp
	if (bmpinfo.biBitCount != 24) {
		CloseHandle(file);
		return NULL;
	}

	// create buffer to hold the data
	size = bmpheader.bfSize - bmpheader.bfOffBits;
	BYTE* Buffer = new BYTE[size];
	// move file pointer to start of bitmap data
	SetFilePointer(file, bmpheader.bfOffBits, NULL, FILE_BEGIN);
	// read bmp data
	if (ReadFile(file, Buffer, size, &bytesread, NULL) == false) {
		delete[] Buffer;
		CloseHandle(file);
		return NULL;
	}
	// everything successful here: close file and return buffer
	CloseHandle(file);

	return Buffer;
}//LOADPMB

void Compute_Projection_Matrix(int point_count, float* world_points, float* image_points, float* A) {
	calib_count = 25;
	MatrixXf D(point_count * 2, 11);
	VectorXf R(point_count * 2);
	float X, Y, Z, x, y;
	for (int i = 0; i < point_count; i++) {
		X = world_points[i * 3];
		Y = world_points[i * 3 + 1];
		Z = world_points[i * 3 + 2];
		x = image_points[i * 2];
		y = image_points[i * 2 + 1];
		R(i * 2) = x;
		R(i * 2 + 1) = y;
		float current_point_data[22] = { X,Y,Z,1,0,0,0,0,-X * x,-Y * x,-Z * x,
			0,0,0,0,X,Y,Z,1,-X * y,-Y * y,-Z * y };
		for (int j = 0; j < 11; j++) {
			D(i * 2, j) = current_point_data[j];
			D(i * 2 + 1, j) = current_point_data[11 + j];
		}
	}

	VectorXf solution(11);
	JacobiSVD<MatrixXf> svd(D, ComputeFullU | ComputeFullV);
	solution = svd.solve(R);

	for (int i = 0; i < 11; i++) {
		A[i] = solution(i);
	}
	A[11] = 1;
}

void Reconstruct(int point_count, float* test1, float* p1, float* w) {
	MatrixXf A(2, 3);
	Vector2f B;
	float x1, y1;
	for (int i = 0; i < point_count; i++) {
		x1 = test1[i * 2];
		y1 = test1[i * 2 + 1];

		A << x1 * p1[8] - p1[0], x1*p1[9] - p1[1], x1*p1[10] - p1[2],
			y1*p1[8] - p1[4], y1*p1[9] - p1[5], y1*p1[10] - p1[6];
		B << p1[3] - x1 * p1[11],
			p1[7] - y1 * p1[11];

		Vector3f solution;
		JacobiSVD<MatrixXf> svd(A, ComputeFullU | ComputeFullV);
		solution = svd.solve(B);
		w[i * 3 + 0] = solution(0);
		w[i * 3 + 1] = solution(1);
		w[i * 3 + 2] = solution(2);
	}
}

namespace emptyformc {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for MyForm
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::MenuStrip^  menuStrip1;
	private: System::Windows::Forms::PictureBox^  pictureBox1;
	private: System::Windows::Forms::OpenFileDialog^  openFileDialog1;
	private: System::Windows::Forms::ToolStripMenuItem^  menuToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  loadBMPToolStripMenuItem;
	private: System::Windows::Forms::RadioButton^  radioButton1;
	private: System::Windows::Forms::RadioButton^  radioButton2;
	private: System::Windows::Forms::RadioButton^  radioButton3;
	private: System::Windows::Forms::RadioButton^  radioButton4;
	private: System::Windows::Forms::RichTextBox^  richTextBox1;
	private: System::Windows::Forms::Button^  button1;
	private: System::Windows::Forms::RichTextBox^  richTextBox2;
	protected:

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->menuStrip1 = (gcnew System::Windows::Forms::MenuStrip());
			this->menuToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->loadBMPToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->openFileDialog1 = (gcnew System::Windows::Forms::OpenFileDialog());
			this->radioButton1 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButton2 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButton3 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButton4 = (gcnew System::Windows::Forms::RadioButton());
			this->richTextBox1 = (gcnew System::Windows::Forms::RichTextBox());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->richTextBox2 = (gcnew System::Windows::Forms::RichTextBox());
			this->menuStrip1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			this->SuspendLayout();
			// 
			// menuStrip1
			// 
			this->menuStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) { this->menuToolStripMenuItem });
			this->menuStrip1->Location = System::Drawing::Point(0, 0);
			this->menuStrip1->Name = L"menuStrip1";
			this->menuStrip1->Size = System::Drawing::Size(975, 24);
			this->menuStrip1->TabIndex = 0;
			this->menuStrip1->Text = L"menuStrip1";
			// 
			// menuToolStripMenuItem
			// 
			this->menuToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) { this->loadBMPToolStripMenuItem });
			this->menuToolStripMenuItem->Name = L"menuToolStripMenuItem";
			this->menuToolStripMenuItem->Size = System::Drawing::Size(50, 20);
			this->menuToolStripMenuItem->Text = L"Menu";
			// 
			// loadBMPToolStripMenuItem
			// 
			this->loadBMPToolStripMenuItem->Name = L"loadBMPToolStripMenuItem";
			this->loadBMPToolStripMenuItem->Size = System::Drawing::Size(128, 22);
			this->loadBMPToolStripMenuItem->Text = L"Load BMP";
			this->loadBMPToolStripMenuItem->Click += gcnew System::EventHandler(this, &MyForm::loadBMPToolStripMenuItem_Click);
			// 
			// pictureBox1
			// 
			this->pictureBox1->Location = System::Drawing::Point(13, 28);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(640, 480);
			this->pictureBox1->TabIndex = 1;
			this->pictureBox1->TabStop = false;
			this->pictureBox1->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::pictureBox1_MouseUp);
			// 
			// openFileDialog1
			// 
			this->openFileDialog1->FileName = L"openFileDialog1";
			// 
			// radioButton1
			// 
			this->radioButton1->AutoSize = true;
			this->radioButton1->Location = System::Drawing::Point(659, 28);
			this->radioButton1->Name = L"radioButton1";
			this->radioButton1->Size = System::Drawing::Size(112, 17);
			this->radioButton1->TabIndex = 2;
			this->radioButton1->TabStop = true;
			this->radioButton1->Text = L"Manuel Calibration";
			this->radioButton1->UseVisualStyleBackColor = true;
			// 
			// radioButton2
			// 
			this->radioButton2->AutoSize = true;
			this->radioButton2->Location = System::Drawing::Point(659, 340);
			this->radioButton2->Name = L"radioButton2";
			this->radioButton2->Size = System::Drawing::Size(72, 17);
			this->radioButton2->TabIndex = 3;
			this->radioButton2->TabStop = true;
			this->radioButton2->Text = L"Point One";
			this->radioButton2->UseVisualStyleBackColor = true;
			// 
			// radioButton3
			// 
			this->radioButton3->AutoSize = true;
			this->radioButton3->Location = System::Drawing::Point(659, 363);
			this->radioButton3->Name = L"radioButton3";
			this->radioButton3->Size = System::Drawing::Size(76, 17);
			this->radioButton3->TabIndex = 4;
			this->radioButton3->TabStop = true;
			this->radioButton3->Text = L"Point Two ";
			this->radioButton3->UseVisualStyleBackColor = true;
			// 
			// radioButton4
			// 
			this->radioButton4->AutoSize = true;
			this->radioButton4->Location = System::Drawing::Point(659, 386);
			this->radioButton4->Name = L"radioButton4";
			this->radioButton4->Size = System::Drawing::Size(83, 17);
			this->radioButton4->TabIndex = 5;
			this->radioButton4->TabStop = true;
			this->radioButton4->Text = L"Point Three ";
			this->radioButton4->UseVisualStyleBackColor = true;
			// 
			// richTextBox1
			// 
			this->richTextBox1->Location = System::Drawing::Point(660, 52);
			this->richTextBox1->Name = L"richTextBox1";
			this->richTextBox1->Size = System::Drawing::Size(100, 96);
			this->richTextBox1->TabIndex = 6;
			this->richTextBox1->Text = L"";
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(659, 154);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(75, 23);
			this->button1->TabIndex = 7;
			this->button1->Text = L"Calibrate";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &MyForm::button1_Click);
			// 
			// richTextBox2
			// 
			this->richTextBox2->Location = System::Drawing::Point(659, 183);
			this->richTextBox2->Name = L"richTextBox2";
			this->richTextBox2->Size = System::Drawing::Size(306, 46);
			this->richTextBox2->TabIndex = 8;
			this->richTextBox2->Text = L"";
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(975, 522);
			this->Controls->Add(this->richTextBox2);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->richTextBox1);
			this->Controls->Add(this->radioButton4);
			this->Controls->Add(this->radioButton3);
			this->Controls->Add(this->radioButton2);
			this->Controls->Add(this->radioButton1);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->menuStrip1);
			this->MainMenuStrip = this->menuStrip1;
			this->Name = L"MyForm";
			this->Text = L"MyForm";
			this->menuStrip1->ResumeLayout(false);
			this->menuStrip1->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void loadBMPToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		if (openFileDialog1->ShowDialog() == System::Windows::Forms::DialogResult::OK) {
			pictureBox1->ImageLocation = openFileDialog1->FileName;
			CString str;
			str = openFileDialog1->FileName;
			input = (LPCTSTR)str;
			buffer = LoadBMP(iWidth, iHeight, iSize, input);
		}
	}
private: System::Void pictureBox1_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	mouse_x = e->X;
	mouse_y = e->Y;
	if (radioButton1->Checked) {
		calib_count++;
		calib_x = mouse_x;
		calib_y = mouse_y;
		richTextBox1->AppendText(mouse_x.ToString() + "     " + mouse_y.ToString() + "     \n");

		StreamWriter^ yaz = gcnew StreamWriter("image points.txt");
		yaz->WriteLine(richTextBox1->Text);
		yaz->Close();
	}
	else if (radioButton2->Checked) {
		p1x = mouse_x;
		p1y = mouse_y;
		label1->Text = "( " + p1x.ToString() + " , " + p1y.ToString() + " )";

		float* test1 = new float[2];
		test1[0] = p2x;
		test1[1] = p2y;
		float* rw = new float[3];
		Reconstruct(1, test1, projection, rw);

		w2x = rw[0];
		w2y = rw[1];
		label5->Text = "( " + w2x.ToString() + " , " + w2y.ToString() + " )";

		delete[] test1;
		delete[] rw;
	}
	else if (radioButton3->Checked) {
		p2x = mouse_x;
		p2y = mouse_y;
		label2->Text = "( " + p2x.ToString() + " , " + p2y.ToString() + " )";

		float* test1 = new float[2];
		test1[0] = p2x;
		test1[1] = p2y;
		float* rw = new float[3];
		Reconstruct(1, test1, projection, rw);

		w2x = rw[0];
		w2y = rw[1];
		label5->Text = "( " + w2x.ToString() + " , " + w2y.ToString() + " )";

		delete[] test1;
		delete[] rw;
	}
	else if (radioButton4->Checked) {
		p3x = mouse_x;
		p3y = mouse_y;
		label3->Text = "( " + p3x.ToString() + " , " + p3y.ToString() + " )";

		float* test1 = new float[2];
		test1[0] = p3x;
		test1[1] = p3y;
		float* rw = new float[3];
		Reconstruct(1, test1, projection, rw);

		w3x = rw[0];
		w3y = rw[1];
		label6->Text = "( " + w3x.ToString() + " , " + w3y.ToString() + " )";

		delete[] test1;
		delete[] rw;
	}
}
private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
	richTextBox2->Clear();

	image = new float[calib_count * 2];
	world = new float[calib_count * 3];

	FILE* imageFile;
	FILE* worldFile;
	imageFile = fopen("calibration image points.txt", "r");
	rewind(imageFile);
	worldFile = fopen("calibration world points.txt", "r");
	rewind(worldFile);

	for (int i = 0; i < calib_count; i++) {
		fscanf(imageFile, "%f", &image[i * 2 + 0]);
		fscanf(imageFile, "%f", &image[i * 2 + 1]);
		fscanf(worldFile, "%f", &world[i * 3 + 0]);
		fscanf(worldFile, "%f", &world[i * 3 + 1]);
		fscanf(worldFile, "%f", &world[i * 3 + 2]);
	}

	fclose(worldFile);
	fclose(imageFile);

	Compute_Projection_Matrix(calib_count, world, image, projection);

	for (int i = 0; i < 3; i++) {
		richTextBox2->AppendText(projection[i * 4 + 0].ToString() + "     " + projection[i * 4 + 1].ToString() + "     " + projection[i * 4 + 2].ToString() + "     " + projection[i * 4 + 3].ToString() + "     \n");
	}

	StreamWriter^ yaz = gcnew StreamWriter("projection.txt");
	yaz->WriteLine(richTextBox2->Text);
	yaz->Close();
}
};
}
